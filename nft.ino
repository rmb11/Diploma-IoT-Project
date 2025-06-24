#include <WiFi.h>
#include <PubSubClient.h>

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

#define AIO_SERVER "io.adafruit.com"
#define AIO_PORT 1883
#define AIO_USERNAME "rmb90"
#define AIO_KEY "" // ---- enter key

#define TOPIC_NUTRIENT_LEVEL AIO_USERNAME "/feeds/nutrient-level"
#define TOPIC_INLET_FLOW_RATE AIO_USERNAME "/feeds/inlet-flow-rate"
#define TOPIC_OUTLET_FLOW_RATE AIO_USERNAME "/feeds/outlet-flow-rate"
#define TOPIC_ALERT_EVENT_LOW_LEVEL AIO_USERNAME "/feeds/alert-event-low-level"
#define TOPIC_ALERT_EVENT_LEAK AIO_USERNAME "/feeds/alert-event-leak"

#define ULTRASONIC_TRIG_PIN 12
#define ULTRASONIC_ECHO_PIN 14
#define INLET_FLOW_SENSOR_PIN 35
#define OUTLET_FLOW_SENSOR_PIN 32

const float NUTRIENT_FULL_THRESHOLD_CM = 5.0;
const float NUTRIENT_LOW_THRESHOLD_CM = 15.0;
const float RESERVOIR_EMPTY_DISTANCE_CM = 50.0;
const float ULTRASONIC_MAX_VALID_RANGE = 400.0;
const int MAX_FLOW_UNITS = 100;
const float LEAK_DIFFERENCE_THRESHOLD = 10.0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
char msgBuffer[50];

unsigned long lastPublishTime = 0;
const unsigned long PUBLISH_INTERVAL = 10000; // 10 seconds

void connectToWiFi();
void reconnectMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);
float readUltrasonicDistance();
float readSimulatedFlowRate(int sensorPin);
void publishSensorData(float distanceCm, float inletFlowRate, float outletFlowRate);
void checkAlertConditions(float distanceCm, float inletFlowRate, float outletFlowRate);

void setup() {
  Serial.begin(115200);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  mqttClient.setServer(AIO_SERVER, AIO_PORT);
  mqttClient.setCallback(mqttCallback);

  connectToWiFi();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }

  mqttClient.loop();

  if (millis() - lastPublishTime >= PUBLISH_INTERVAL) {
    lastPublishTime = millis();

    float distanceCm = readUltrasonicDistance();
    float inletFlowRate = readSimulatedFlowRate(INLET_FLOW_SENSOR_PIN);
    float outletFlowRate = readSimulatedFlowRate(OUTLET_FLOW_SENSOR_PIN);

    publishSensorData(distanceCm, inletFlowRate, outletFlowRate);
    checkAlertConditions(distanceCm, inletFlowRate, outletFlowRate);

    Serial.println("-----------------------------------");
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi. Retrying...");
  }
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), AIO_USERNAME, AIO_KEY)) {
      Serial.println("connected");
      // Optional: mqttClient.subscribe("some/command/feed");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

float readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  long timeoutMicroseconds = (long)(RESERVOIR_EMPTY_DISTANCE_CM * 2 / 0.0343) + 2000;
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, timeoutMicroseconds);
  float distanceCm = duration * 0.0343 / 2;

  Serial.print("Ultrasonic raw duration: ");
  Serial.print(duration);
  Serial.print(" us, Calculated Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  if (duration == 0) {
    Serial.println("WARN: Ultrasonic sensor timed out (no echo). Interpreting as empty/error.");
    return RESERVOIR_EMPTY_DISTANCE_CM + 5.0;
  }
  if (distanceCm >= RESERVOIR_EMPTY_DISTANCE_CM - 2.0) {
    Serial.println("INFO: Ultrasonic sensor indicates reservoir is empty/critically low.");
    return RESERVOIR_EMPTY_DISTANCE_CM;
  }
  if (distanceCm > ULTRASONIC_MAX_VALID_RANGE + 10.0) {
    Serial.println("CRITICAL: Ultrasonic sensor reading highly out of realistic range. Sensor error?");
    return -1.0;
  }

  return distanceCm;
}

float readSimulatedFlowRate(int sensorPin) {
  int rawValue = analogRead(sensorPin);
  float flowRate = map(rawValue, 0, 4095, 0, MAX_FLOW_UNITS);

  Serial.print("Pin ");
  Serial.print(sensorPin);
  Serial.print(" Raw: ");
  Serial.print(rawValue);
  Serial.print(" | Flow Rate: ");
  Serial.print(flowRate);
  Serial.println(" units/min");

  return flowRate;
}

void publishSensorData(float distanceCm, float inletFlowRate, float outletFlowRate) {
  if (mqttClient.connected()) {
    if (distanceCm >= 0) {
      dtostrf(distanceCm, 4, 2, msgBuffer);
      mqttClient.publish(TOPIC_NUTRIENT_LEVEL, msgBuffer);
      Serial.print("SENT: "); Serial.print(TOPIC_NUTRIENT_LEVEL); Serial.print(" -> "); Serial.println(msgBuffer);
    }

    dtostrf(inletFlowRate, 4, 1, msgBuffer);
    mqttClient.publish(TOPIC_INLET_FLOW_RATE, msgBuffer);
    Serial.print("SENT: "); Serial.print(TOPIC_INLET_FLOW_RATE); Serial.print(" -> "); Serial.println(msgBuffer);

    dtostrf(outletFlowRate, 4, 1, msgBuffer);
    mqttClient.publish(TOPIC_OUTLET_FLOW_RATE, msgBuffer);
    Serial.print("SENT: "); Serial.print(TOPIC_OUTLET_FLOW_RATE); Serial.print(" -> "); Serial.println(msgBuffer);

  } else {
    Serial.println("WARN: MQTT client not connected, cannot publish data.");
  }
}

void checkAlertConditions(float distanceCm, float inletFlowRate, float floatoutletFlowRate) {
  bool nutrientLowOrEmpty = false;
  bool leakDetected = false;

  if (distanceCm >= 0) {
    if (distanceCm >= RESERVOIR_EMPTY_DISTANCE_CM - 2.0) {
      Serial.println("ALERT: Reservoir is empty or critically low!");
      nutrientLowOrEmpty = true;
    } else if (distanceCm > NUTRIENT_LOW_THRESHOLD_CM) {
      Serial.println("ALERT: Nutrient Solution Level is LOW!");
      nutrientLowOrEmpty = true;
    }
  }

  if (mqttClient.connected() && nutrientLowOrEmpty) {
    mqttClient.publish(TOPIC_ALERT_EVENT_LOW_LEVEL, "LOW_LEVEL_DETECTED");
    Serial.print("SENT: "); Serial.print(TOPIC_ALERT_EVENT_LOW_LEVEL); Serial.println(" -> LOW_LEVEL_DETECTED");
  }

  if (inletFlowRate > floatoutletFlowRate + LEAK_DIFFERENCE_THRESHOLD) {
    Serial.println("ALERT: Potential Leak Detected! Investigate immediately.");
    leakDetected = true;
    if (mqttClient.connected()) {
      mqttClient.publish(TOPIC_ALERT_EVENT_LEAK, "LEAK_DETECTED");
      Serial.print("SENT: "); Serial.print(TOPIC_ALERT_EVENT_LEAK); Serial.println(" -> LEAK_DETECTED");
    }
  }
}
