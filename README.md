
# 🤖 Gesture-Controlled IoT Robot

This project demonstrates a **gesture-controlled robot** that can be operated **wirelessly via hand gestures** using an **MPU6050 sensor glove**, and also **remotely via the internet** using **Blynk IoT Cloud**.
It combines **ESP-NOW**, **UDP communication**, and **Wi-Fi (Blynk IoT)** for hybrid peer-to-peer and cloud-based control.

---

## 🧠 System Overview

The project consists of **two main units**:

1. **Glove Controller (ESP8266 NodeMCU)**

   * Reads hand gestures using the **MPU6050** sensor (accelerometer + gyroscope).
   * Uses **I²C** protocol to communicate with the MPU6050.
   * Sends gesture data to the robot via **ESP-NOW**.
   * Uses **UDP broadcast** over Wi-Fi to detect and pair with the robot before switching to ESP-NOW.
   * Includes **mode-switch buttons** to toggle between **car control** and **arm control**.

2. **Robot Unit (ESP32-WROOM-DA)**

   * Receives gesture data and controls the **DC motors** (for car movement) and **servo motors** (for robotic arm).
   * Uses **ultrasonic sensor (HC-SR04)** for obstacle detection.
   * Connects to **Blynk IoT Cloud** for remote monitoring and control.
   * Supports automatic switching between **Wi-Fi (IoT mode)** and **ESP-NOW (gesture mode)**.

---

## ⚙️ Features

* Real-time gesture control using MPU6050.
* Dual communication: ESP-NOW + Wi-Fi.
* IoT control & monitoring via **Blynk Cloud**.
* Automatic robot-glove pairing via **UDP broadcast**.
* Ultrasonic obstacle detection.
* Power-efficient with 4×3.7 V Li-ion battery pack.
* Robotic arm with 5 servo motors.
* Dual antenna ESP32-WROOM-DA for improved range.

---

## 🧩 Hardware Components

| Component                        | Quantity | Purpose                  |
| -------------------------------- | -------- | ------------------------ |
| **ESP8266 NodeMCU**              | 1        | Glove controller         |
| **ESP32-WROOM-DA**               | 1        | Robot controller         |
| **MPU6050**                      | 1        | Gesture detection sensor |
| **Ultrasonic Sensor (HC-SR04)**  | 1        | Obstacle detection       |
| **Servo Motors**                 | 5        | Robotic arm joints       |
| **DC Motors**                    | 4        | Car movement             |
| **Motor Driver (L298N / L293D)** | 1        | DC motor control         |
| **Mode Switch Buttons**          | 2        | Gesture mode switching   |
| **Li-ion 3.7 V Batteries**       | 4        | Power supply             |
| **LEDs**                         | 2–3      | Status indication        |

---

## 🧠 Communication & Protocols

| Protocol           | Used Between        | Purpose                                |
| ------------------ | ------------------- | -------------------------------------- |
| **I²C**            | ESP8266 ↔ MPU6050   | Sensor data reading                    |
| **ESP-NOW**        | ESP8266 ↔ ESP32     | Peer-to-peer gesture data transmission |
| **UDP**            | ESP8266 → ESP32     | Device discovery & pairing             |
| **Wi-Fi (802.11)** | ESPs ↔ Router       | Internet & Blynk Cloud access          |
| **MQTT / HTTP**    | ESP32 ↔ Blynk Cloud | IoT dashboard data exchange            |

---

## 🧰 Software & Libraries

| Software                    | Purpose                      |
| --------------------------- | ---------------------------- |
| **Arduino IDE**             | Code development & uploading |
| **Blynk IoT Console / App** | IoT dashboard & control      |
| **Serial Monitor**          | Debug & test communication   |

**Libraries:**

* `Wire.h` → I²C communication
* `MPU6050.h` → Read accelerometer & gyroscope data
* `ESP8266WiFi.h` / `WiFi.h` → Wi-Fi management
* `WiFiUdp.h` → UDP broadcast & discovery
* `esp_now.h` → Peer-to-peer communication
* `BlynkSimpleEsp32.h` / `BlynkSimpleEsp8266.h` → Blynk IoT integration
* `Servo.h` → Control servo motors
* `NewPing.h` / `Ultrasonic.h` → Distance measurement
* `mDNS.h` → Local device discovery (optional)

---

## ⚡️ Circuit Connections

### **Glove (ESP8266 + MPU6050 + Buttons)**

| Device        | ESP8266 Pin |
| ------------- | ----------- |
| MPU6050 SDA   | D2 (GPIO4)  |
| MPU6050 SCL   | D1 (GPIO5)  |
| Mode Button 1 | D6 (GPIO12) |
| Mode Button 2 | D7 (GPIO13) |
| LED Indicator | D3 (GPIO0)  |

### **Robot (ESP32-WROOM-DA)**

| Device               | ESP32 Pin               | Description         |
| -------------------- | ----------------------- | ------------------- |
| Motor Driver (L298N) | GPIO 18, 19, 23, 25     | DC motor control    |
| Servo Motors         | GPIO 26, 27, 32, 33, 14 | Robotic arm control |
| Ultrasonic Trigger   | GPIO 5                  | Output              |
| Ultrasonic Echo      | GPIO 4                  | Input               |
| Status LED           | GPIO 2                  | Indicator           |

---

## ☁️ Blynk IoT Setup

1. Create a new device on **[Blynk IoT Console](https://blynk.cloud/)**.
2. Copy the **Auth Token** and paste it in your ESP32 robot code:

   ```cpp
   char auth[] = "YourAuthTokenHere";
   ```
3. Add widgets:

   | Widget          | Virtual Pin | Purpose                              |
   | --------------- | ----------- | ------------------------------------ |
   | Gauge           | V0          | Battery Level                        |
   | Graph           | V1          | Ultrasonic Distance                  |
   | Button / Slider | V2–V6       | Manual control for movement & servos |
   | LED Widget      | V7          | Connection indicator                 |

---

## 🛠️ Setup & Configuration

1. **Install Arduino IDE**

   * Add ESP8266 & ESP32 board support from Board Manager.
   * Install required libraries via Library Manager.

2. **Edit Wi-Fi Credentials**

   ```cpp
   const char* ssid = "YourWiFiName";
   const char* password = "YourWiFiPassword";
   ```

3. **Configure Blynk**

   * Paste your **Blynk Auth Token** in the ESP32 code.

4. **Upload Code**

   * Upload `Glove_ESP8266.ino` to ESP8266 NodeMCU.
   * Upload `Robot_ESP32.ino` to ESP32-WROOM-DA.

5. **Power On**

   * Glove connects to Wi-Fi → sends UDP `"GLOVE_ON"` message.
   * Robot receives UDP → replies `"ROBOT_ACK"` → both switch to ESP-NOW mode.
   * Robot starts receiving gesture data.

---

## 🔍 Troubleshooting

| Issue                            | Cause                       | Fix                                           |
| -------------------------------- | --------------------------- | --------------------------------------------- |
| Robot not responding to gestures | UDP pairing failed          | Check Wi-Fi SSID & same network               |
| Blynk not updating               | Incorrect Auth Token        | Recheck Blynk configuration                   |
| Servo jitter                     | Insufficient power          | Use dedicated 5V supply or 4-cell Li-ion pack |
| ESP32 not uploading              | Wrong COM port or boot mode | Hold BOOT button while uploading              |

---

## 📊 System Architecture

```
        ┌──────────────┐
        │   MPU6050    │
        └──────┬───────┘
               │ (I²C)
        ┌──────▼──────┐
        │  ESP8266    │  ← Glove
        │ (Gesture Tx)│
        └──────┬──────┘
               │ ESP-NOW / UDP
        ┌──────▼──────┐
        │  ESP32-WROOM│  ← Robot
        │ (Motor + IoT)│
        ├──────┬───────┤
        │ Blynk IoT ☁️ │
        └──────────────┘
```

---

## 🧑‍💻 Authors

👨‍🔧 **Janidu Chamika** <br/>
👨‍🔧 **Chaami Siriwardhena**<br/>
**Department of Statistics & Computer Science, University of Kelaniya**

## 📜 License

This project is released under the **MIT License** — you’re free to modify and use it for educational or personal projects.


