# NFT-Hydroponics System for Monitoring Leaks and Low Levels in Reservoir (Simulation Prototype)

## Project Overview

Welcome to the NFT Hydroponics Monitoring System project!

This system monitors nutrient levels and detect possible leaks in an NFT hydroponics setup in real-time. It provides alerts to a dashboard and email and eliminates the need for constant manual checks in the greenhouse. 

---

## Features

* **Real-time nutrient levels:** Uses an ultrasonic sensor to track nutrient solution levels.
* **Real-time leak detection:** Uses two flow sensors to detect any discrepencies which could indicate a leak. 
* **Automatic Low Level Alerts:** Sends email notifications if the simulated nutrient solution level dips below 15cm.
* **Automatic Leak Alerts:** Sends email notifications if a possible leak is detected. 
* **Cloud-based Dashboard:** Provides remote access to real-time and historical trends on Adafruit IO.
* **Alert History:** Logs all important events for review and analysis.
* **Basic Error Handling:** Includes checks for possible sensor malfunctions and connectivity issues.

---

## Hardware Components

This section outlines the virtual hardware components used in the Wokwi simulation. 

### Microcontroller

* **ESP32-S2-DEVKIT** 

### Sensors

* **JSN-SR04T Waterproof Ultrasonic Sensor (Simulated):**
* **YF-S201 Hall-effect Water Flow Sensors (x2) (Simulated):**

### Essentials (Conceptual for Simulation)

* **Breadboard & Jumper Wires:** Represent the virtual connections in Wokwi.
* **Logic Level Shifters (5V to 3.3V, x2) or Resistors for Voltage Dividers:** *Important design consideration for actual hardware implementation.* 

---

## Cloud Setup (Adafruit IO)

This project uses [Adafruit IO](https://io.adafruit.com/) for data management and alerts. You will need an Adafruit IO account.

### Feeds

Create the following 5 feeds in your Adafruit IO account and **ensure the "key" for each feed matches exactly as listed. 

| Feed Key | Description |
| :------------------- | :----------------------------------- |
| `nutrient-level` | Nutrient tank level (cm) |
| `inlet-flow-rate` | Inlet flow (units/min) |
| `outlet-flow-rate` | Outlet flow (units/min) |
| `alert-event-low-level` | Low nutrient alert event |
| `alert-event-leak` | Leak detected alert event |

### Dashboard

Create a dashboard on Adafruit IO to visualize your data:

1.  **Create a new dashboard:** on Adafruit IO.
2.  **Add blocks:**
    * **Gauges & Line Charts** for:
        * `nutrient-level`
        * `inlet-flow-rate`
        * `outlet-flow-rate`
        * *Configure Min/Max values and Unit for gauges (e.g., Nutrient Level Gauge: Min 0, Max 50, Unit 'cm').*
    * **Stream** block for:
        * `alert-event-low-level` and `alert-event-leak` (to show alert history)

### Alert Triggers (Email)

Set up email alerts to notify you when critical conditions are met:

1.  **Go to "Actions" (in the top navigation bar) -> "Triggers"** on Adafruit IO.
2.  **Create a New Trigger:**
    * **For Low Nutrient Alert:**
        * **Name:** `Email Alert Low Levels`
        * **WHEN:** `Default: alert-event-low-level gets data matching = "LOW_LEVEL_DETECTED"`
        * **THEN (Action):** `Email`
            * **Recipients:** Your email address(es)
            * **Subject:** `Low Nutrient Warning`
            * **Message:** `Nutrient level is low. Check system!`
    * **For Leak Detected Alert:**
        * **Name:** `Email Alert Leak Detected`
        * **WHEN:** `Default: alert-event-leak gets data matching = "LEAK_DETECTED"`
        * **THEN (Action):** `Email`
            * **Recipients:** Your email address(es)
            * **Subject:** `Leak detected!`
            * **Message:** `Possible leak detected. Check system!`

---

## Getting Started (Wokwi Simulation)

This guide helps you run the project in the Wokwi online simulator.

### What You'll Need

* Internet connection and web browser.
* Adafruit IO account.

### Wiring It Up (Simulation)

In Wokwi, the components are already wired for you in the diagram. 

| Component | ESP32 GPIO |
| :---------------- | :--------- |
| JSN-SR04T TRIG | `GPIO 12` |
| JSN-SR04T ECHO | `GPIO 14` |
| Flow Sensor In | `GPIO 35` |
| Flow Sensor Out | `GPIO 32` |

### Code Setup

1.  **Open the Sketch:** Open the provided `nft.ino` file in Arduino IDE.
2.  **Update Your Credentials:** in the following:

    ```cpp
    const char* WIFI_SSID = "Wokwi-GUEST"; // Default Wokwi Wi-Fi
    const char* WIFI_PASSWORD = "";        // Default Wokwi Wi-Fi password (none)

    #define AIO_USERNAME "YOUR_ADAFRUIT_IO_USERNAME" // Your Adafruit IO Username
    #define AIO_KEY "YOUR_ADAFRUIT_IO_KEY"           // Your Adafruit IO Key 
    ```

3.  **Change Thresholds (Optional):**
    * `NUTRIENT_LOW_THRESHOLD_CM`: Adjust this if the "low" level is different.
    * `RESERVOIR_EMPTY_DISTANCE_CM`: Distance from the simulated sensor to the bottom of the reservoir.
    * `LEAK_DIFFERENCE_THRESHOLD`: Adjust if you want a different sensitivity for leak detection.

---

## How It Works

After updating the code in Wokwi, click the "Play" button to start the simulation. 

### Operation

* The system will read the sensor data and send it to Adafruit IO every 15 seconds.
* It will try to reconnect if the Wi-Fi or MQTT connection drops.
* View the dashboard in Adafruit IO to see real time data. 

---

## License

This project is open source and shared under the [MIT License]. See `LICENSE.md` file for details.
