# 🌱 Smart Agriculture CAN Communication System

This project implements a **FreeRTOS-based system** for smart agriculture, where a **STM32 microcontroller** sends soil/environment sensor data over **CAN bus** to an **ESP32-C3**, which displays the data, controls actuators (pump and fan), and uploads everything to **Grafana Cloud** via **InfluxDB**.

---

## 📤 Sender (STM32 with MCP2515)

### 🔧 Hardware

* STM32F103C8T6 (Blue Pill)
* MCP2515 CAN module (8MHz crystal)
* DHT11 sensor (air temperature + humidity)
* Soil moisture sensor (analog)
* Power: 3.3V / 5V (logic-level compatible with MCP2515)

### 📦 Dependencies

* `MapleFreeRTOS900`
* `SPI`
* `mcp2515` library
* `DHT` library

### 🚀 Features

* Periodically reads:

  * Soil moisture (analog)
  * Air temperature and humidity (DHT11)
* Converts soil moisture ADC value to percentage
* Packs data into an 8-byte CAN frame:

  ```
  [ADC_L, ADC_H, Soil%, Air Humidity, Temperature, 0, 0, 0]
  ```
* Sends the frame every 2 seconds using FreeRTOS task

### 🧵 Tasks

* `taskReadSensors`: Reads and stores data in a shared struct
* `taskSendCAN`: Sends CAN messages based on latest sensor readings

---

## 📥 Receiver (ESP32-C3 with MCP2515)

### 🔧 Hardware

* ESP32-C3 Dev Board
* MCP2515 CAN module (8MHz)
* OLED SSD1306 display (I2C)
* L298N motor driver

  * One output for **water pump**
  * One output for **fan (reversible)**
* WiFi for Grafana Cloud connection

### 📦 Dependencies

* `Adafruit_SSD1306`, `Adafruit_GFX`
* `WiFi.h`
* `InfluxDbClient.h`
* `freertos/FreeRTOS.h` and related headers

### 🚀 Features

* Receives and decodes CAN messages
* Displays live data on OLED
* Controls:

  * **Water pump** based on soil moisture:

    * `< 20%`: full speed
    * `20–40%`: low speed
    * `>= 40%`: off
  * **Fan** based on temp & humidity:

    * `Temp > 30°C & Hum > 80%`: reverse
    * `Temp > 30°C & Hum < 50%`: forward
    * `25–30°C & Hum > 80%`: reverse
    * `Temp < 25°C & Hum < 50%`: off
* Uploads sensor data + actuator states to **Grafana via InfluxDB Cloud**

### 🧵 Tasks

* `readCANTask`: Reads incoming CAN frames
* `controlMotorTask`: Controls pump motor
* `controlFanTask`: Controls fan direction
* `displayTask`: Shows data on OLED
* `grafanaTask`: Sends data to InfluxDB every 10s

---

## 📡 Grafana Dashboard

* Displays:

  * Soil moisture (%)
  * Air temperature (°C)
  * Air humidity (%)
  * Water pump speed (PWM)
  * Fan status (OFF / FORWARD / REVERSE)
* Backend: **InfluxDB Cloud**
* Frontend: **Grafana Cloud**

---

## 🖼 System Architecture

```
[Sensors] ─> STM32 ──> CAN bus ──> ESP32-C3 ──> [Display + Actuators + WiFi → Grafana]
```

---

## ⚠️ Notes

* MCP2515 modules must be properly grounded and terminated (120Ω resistor)
* Ensure voltage compatibility between STM32 (3.3V) and MCP2515 (typically 5V)
* Use logic-level shifters if necessary
* Check DHT11 read error handling (NaN cases)
