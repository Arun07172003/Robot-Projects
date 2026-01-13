# Robot-Projects
Multi mode mobile robot using a single hardware platform with Wi-Fi control, line following, and human following implemented through modular firmware

# Multi-Mode Mobile Robot 

This repository contains multiple mobile robot applications developed using a **common hardware platform**.  
Different robot behaviors are achieved by changing firmware, without modifying the hardware.

##  Robot Modes Implemented
- **Human Following Robot**
- **Line Following Robot**
- **Wi-Fi Controlled Car**

Each mode is implemented as a separate project/folder for clarity and modularity.

## Key Concept
> One hardware platform → Multiple robot behaviors using software control.
> 
## Hardware Components Used
- **Microcontroller:** ESP32
- **Motor Driver:** L298N Dual H-Bridge Motor Driver
- **Motors:**  
  - 2 × DC Gear Motors  
  - 1 × Servo Motor
- **Sensors:**  
  - Ultrasonic Sensor (for distance measurement & human detection)  
  - 3 × IR Sensors (for line following)
- **Power Management:**  
  - Buck Converter (voltage regulation)
- **Other Components:**  
  - Switches  
  - LEDs  
  - Wheels, chassis, battery, resistors(220 ohm, 1k ohm)

##  Software & Tools
- **Framework:** Arduino (ESP32)
- **IDE:** PlatformIO (VS Code)
- **Programming Language:** C / C++
- **Version Control:** Git & GitHub
