# PX4 Autopilot with Fault Detection and Simulation Plugin

This repository hosts a modified version of the PX4 Autopilot firmware. The modifications introduce a fault detection system that provides specific actuator information and includes a fault simulation plugin to test these capabilities.

## Key Modifications
1. **Fault Detection System** - Enhanced to identify faults in specific actuators, enabling targeted fault diagnosis and improved troubleshooting capabilities.
2. **Fault Simulation Plugin** - A plugin that simulates actuator faults for testing purposes, facilitating validation of the fault detection mechanisms in a controlled environment.

## Requirements
This modified PX4 Autopilot requires:
- **ROS 2 Humble** for integration with ROS 2-based systems.
- **PX4 Firmware Dependencies** as per the official PX4 Autopilot setup.

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/ChinChinati/PX4-Autopilot.git
cd PX4-Autopilot
```

### 2. Install Dependencies Install required dependencies, including PX4 and ROS 2 packages.
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions ros-humble-desktop
```

### 3. Build PX4 Firmware
```bash
cd PX4-Autopilot
make px4_sitl gazebo-classic
```



