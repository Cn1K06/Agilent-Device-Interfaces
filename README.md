# Description
This repository contains Python **and C++** scripts for the remote control of Agilent measurement devices.  
It is designed to work with **Raspberry Pi** and **Ubuntu**, allowing seamless communication with measurement devices via **USB** or **LAN cable**.

It also includes **ROS 2 interfaces (C++)** for integrating Agilent devices into robotic systems and automated workflows.

---

## Key Features

- Send SCPI commands to Agilent devices via Python or ROS 2 interface.
- Perform and save measurements from connected devices.
- ROS 2 packages implemented in **C++** for modular device control.
- Usage documentation for both **Python** and **ROS 2** interfaces.
- Support for Raspberry Pi, Ubuntu 22.04.5 LTS (Jammy Jellyfish) and Ubuntu 24.04 LTS (Jazzy) environments.

---

## Requirements

### Python Environment

- Python 3.7 or higher

**Required Python libraries:**
- pyvisa
- socket
- telnetlib
- time
- threading

---

### ROS 2 Environment

- ROS 2 Jazzy (or compatible version)
- C++17 compiler
- CMake and colcon tools for building ROS 2 packages

**ROS 2 Packages Included:**
- `agilent_34410a_interface`
- `agilent_3458a_interface`
- `agilent_33220a_interface`


**How to Use ROS 2 Interfaces:**
Documentation is provided under each How to use ROS2 Interface and includes:
- Launch instructions
- Available service calls
- Example command usage

---

## Supported Devices

- Agilent 34110A  
- Agilent 3458A  
- Agilent 33220A  
- Agilent 53132A  

---

## Hardware & OS

- USB or LAN cable for connection  
- Raspberry Pi (recommended model: 4B)  
- Ubuntu 22.04.5 LTS (Jammy Jellyfish)  