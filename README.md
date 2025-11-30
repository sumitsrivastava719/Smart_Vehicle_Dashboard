# 🚗 Smart Vehicle Dashboard  
**A Dual-Microcontroller Real-Time Automotive Monitoring System**

This project implements a smart vehicle dashboard capable of real-time data acquisition, inter-MCU communication, visualization, and remote IoT monitoring.  
It uses **STM32F407VGT6**, **ESP32**, **CAN Bus**, **FreeRTOS**, and multiple sensors to simulate a modern automotive dashboard.

---

## 🔧 System Overview

The system follows a **dual-MCU architecture** for modularity, speed, and reliability:

### **1️⃣ STM32F407VGT6 – Primary Controller**
- Handles real-time sensor acquisition  
- Runs multiple FreeRTOS tasks  
- Sends processed data to ESP32 via CAN bus  

### **2️⃣ ESP32-WROOM-32 – Secondary Controller**
- Receives CAN frames from STM32  
- Reads additional environmental sensors  
- Displays dashboard on OLED  
- Uploads full dataset to **Blynk Cloud**  

---

## 📡 Sensors Used & Purpose

### **Sensors connected to STM32**

| Sensor | Purpose |
|-------|---------|
| **LM35** | Measures engine/ambient temperature (analog) |
| **HC-SR04 Ultrasonic** | Measures obstacle distance (2–400 cm) |
| **HC-89 IR Speed Sensor** | Measures rotational speed (RPM) |
| **MPU6050** | Detects jerk, tilt, acceleration changes |

---

### **Sensors connected to ESP32**

| Sensor | Purpose |
|-------|---------|
| **DHT11** | Measures cabin temperature & humidity |
| **MQ-135** | Measures air quality (CO₂, NH₃, NOx, smoke) |
| **NEO-6M GPS** | Provides coordinates, speed & satellite data |
| **SSD1306 OLED** | Displays dashboard metrics |

---

## 🧩 Hardware Setup

<p align="center">
  <img width="700" src="https://github.com/user-attachments/assets/02a7bdad-6376-4828-a46d-1d316e0165b2" alt="Hardware Overview">
</p>

---


## ⚡ Circuit Diagram

<p align="center">
<img width="700" height="659" alt="image" src="https://github.com/user-attachments/assets/fd98a25a-d4cc-4a0c-a3d2-f3824df71d11" />
</p>


---

## 🖧 Communication & Interfaces

### **CAN Bus (STM32 ↔ ESP32)**
Used for high-reliability inter-controller communication.

**CAN Frame IDs**
| ID | Data |
|----|------|
| `0x100` | Temperature |
| `0x101` | Distance |
| `0x102` | Jerk Detection |
| `0x103` | Speed |

### **Other Interfaces**
- **ADC →** LM35, MQ-135  
- **I2C →** SSD1306 OLED, MPU6050  
- **UART →** GPS  
- **GPIO →** Ultrasonic, DHT11, HC-89  

---

## ⚙️ FreeRTOS Task Architecture

### **STM32 Tasks**
- Temperature reading (LM35)  
- Speed measurement (HC-89)  
- Jerk detection (MPU6050)  
- Ultrasonic distance measurement  
- CAN frame packaging & transmission  

### **ESP32 Tasks**
- CAN frame reception  
- DHT11 reading  
- MQ-135 air quality reading  
- GPS parsing (UART)  
- OLED dashboard update  
- Blynk Cloud upload  

---

## 🔄 System Workflow


<p align="center">
<img width="185" height="344" alt="image" src="https://github.com/user-attachments/assets/bf844903-534c-4dd8-b3c3-480a54b9d2bc" />

</p>


1. **STM32 reads sensors using FreeRTOS tasks**  
2. **STM32 packs data → sends via CAN to ESP32**  
3. **ESP32 reads its own sensors (DHT11, MQ-135, GPS)**  
4. **ESP32 combines all data & updates OLED dashboard**  
5. **ESP32 uploads dataset to Blynk Cloud for remote access**

---

## 📊 Project Results

### ✔️ Achievements
- Stable **CAN communication** between STM32 & ESP32  
- Real-time dashboard displaying:
  - Speed  
  - Distance  
  - Engine temperature  
  - Cabin temperature/humidity  
  - Air Quality Index  
  - Jerk detection  
  - GPS coordinates & speed  
- Data accessible remotely via **Blynk**  
- FreeRTOS tasks run concurrently without conflicts  
- System tested continuously for **3+ hours** without failure  

### 📌 Tested Parameter Ranges
| Parameter | Range |
|----------|--------|
| Speed | 0–50 RPM |
| Distance | 5–200 cm |
| Engine Temperature | 25–75°C |
| Humidity | ~50% RH |
| Air Quality | Varying analog response |
| GPS | Live position + speed |

---

## 📦 Components Used
- STM32F407VGT6 Discovery Board  
- ESP32-WROOM-32  
- MCP2515 CAN Controller  
- SN65HVD230 CAN Transceiver  
- LM35, HC-SR04, HC-89, MPU6050  
- MQ-135, DHT11, NEO-6M GPS  
- SSD1306 OLED (128×64)  

---

## ✔️ Conclusion

This project demonstrates a fully functional automotive-style dashboard that integrates:

- Multi-sensor data acquisition  
- CAN-based inter-MCU communication  
- Real-time FreeRTOS task handling  
- Local OLED visualization  
- Cloud-based IoT monitoring  

It forms a strong base for future upgrades such as touch UI, OBD-II integration, battery monitoring, or predictive maintenance.

---
## Link
- 🐙 [GitHub](https://github.com/sumitsrivastava719)
- 🔗 [Linkedin](https://www.linkedin.com/in/-sumitsrivastava-/) 

