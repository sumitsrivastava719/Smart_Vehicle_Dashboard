# Smart Vehicle Dashboard

A Dual-Microcontroller Real-Time Automotive Monitoring System

This project implements a smart vehicle dashboard capable of acquiring, transmitting, visualizing, and remotely monitoring real-time vehicle and environmental parameters. It uses STM32F407VGT6, ESP32, CAN bus, and FreeRTOS, along with multiple sensors to replicate the workflow of a modern automotive dashboard.

🔧 System Overview

The system follows a dual-MCU architecture for modularity and reliability:


1. STM32F407VGT6 (Primary Controller)

Handles all real-time sensor acquisition

Runs multiple FreeRTOS tasks

Transmits processed data to ESP32 via CAN bus


2. ESP32-WROOM-32 (Secondary Controller)

Receives CAN frames from STM32

Collects additional environmental data

Renders a full dashboard on an OLED display

Uploads consolidated data to Blynk Cloud for remote monitoring

📡 Sensors Used & Their Purpose

Sensors connected to STM32


LM35	Measures engine/ambient temperature (analog).

HC-SR04 Ultrasonic	Measures obstacle distance (2–400 cm).

HC-89 IR Speed Sensor	Measures rotational speed (RPM) for vehicle speed.

MPU6050	Detects jerk, tilt, or sudden acceleration changes.


Sensors connected to ESP32

DHT11	Measures cabin temperature and humidity.

MQ-135	Measures air quality (CO₂, NH₃, NOx, smoke, pollutants).

NEO-6M GPS	Provides live coordinates, speed, and satellite data.

SSD1306 OLED	Displays real-time dashboard metrics.



🖧 Communication & Interfaces

CAN Bus (STM32 ↔ ESP32)

Used for high-reliability data transmission between microcontrollers.

CAN Frame IDs used:

0x100 – Temperature

0x101 – Distance

0x102 – Jerk Detection

0x103 – Speed

Other Interfaces

ADC → LM35, MQ-135

I2C → OLED, MPU6050

UART → GPS

Digital GPIO → Ultrasonic, DHT11, HC-89


⚙️ FreeRTOS Task Architecture
STM32 FreeRTOS Tasks

Temperature reading (LM35)

Speed measurement (HC-89)

Jerk detection (MPU6050)

Ultrasonic distance measurement

CAN frame packaging & transmission

ESP32 FreeRTOS Tasks

CAN frame reception

DHT11 reading

Air quality reading (MQ-135)

GPS parsing

OLED display update

Cloud upload (Blynk)


🔄 System Workflow

STM32 continuously reads all connected sensors using FreeRTOS tasks.

STM32 packages values into CAN frames and sends them to ESP32.

ESP32 reads its own sensors (DHT11, MQ-135, GPS).

ESP32 merges both data streams and displays them on the OLED.

ESP32 uploads full dashboard data to Blynk Cloud over Wi-Fi.


📊 Project Results

Fully stable CAN communication between STM32 & ESP32

Real-time dashboard display of:

Speed

Distance

Engine temperature

Cabin temperature/humidity

Air Quality Index

Jerk detection

GPS coordinates & speed

All values accessible remotely via Blynk mobile dashboard

FreeRTOS tasks run concurrently without timing issues

System tested continuously for 3+ hours without failure

Tested Parameter Ranges:

Speed: 0–50 RPM

Distance: 5–200 cm

Engine temperature: 25–75°C

Humidity: around 50% RH

Air quality: fluctuating analog values

GPS: live position + speed



📦 Components Used

STM32F407VGT6 Discovery Board

ESP32-WROOM-32

MCP2515 CAN Controller

SN65HVD230 CAN Transceiver

LM35, HC-SR04, HC-89, MPU6050

MQ-135, DHT11, NEO-6M GPS

SSD1306 OLED Display (128×64)



✔️ Conclusion

This project demonstrates a complete working automotive-style dashboard integrating multi-sensor data acquisition, CAN-based inter-MCU communication, real-time FreeRTOS scheduling, local OLED visualization, and cloud-based IoT monitoring.
It provides a strong foundation for future upgrades such as touch-based UI, OBD-II integration, battery health monitoring, or predictive maintenance.
## Links



- 🐙 [GitHub Repository](https://github.com/sumitsrivastava719/Smart_Vehicle_Dashboard)
- 🔗 [Linkedin](https://www.linkedin.com/in/-sumitsrivastava-/)

