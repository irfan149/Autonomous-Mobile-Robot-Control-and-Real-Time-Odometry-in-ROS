# Autonomous Mobile Robot Control and Real-Time Odometry in ROS

This project implements a mobile robot control system using ROS (Robot Operating System) with real-time odometry feedback and visualization in RViz. The robot is modeled as a differential-drive robot, with motor control and encoder feedback being managed through ROS nodes.

## Features:
- Motor Control: The robot receives velocity commands from ROS for left and right motors.
- Encoder Feedback: The system sends encoder pulse counts to ROS, enabling real-time odometry calculation.
- RViz Visualization: Robot movement is visualized in RViz, including its odometry data.

## Getting Started

### Prerequisites:
- ROS (Robot Operating System): Make sure you have ROS (Melodic, Noetic, etc.) installed on your system.
- Arduino IDE: You will need the Arduino IDE to upload code to the robot's microcontroller.
- rosserial_arduino: ROS package for Arduino communication (`rosserial`).

### Installation:

Clone this repository:
   ```bash
   git clone https://github.com/irfan149/Autonomous-Mobile-Robot-Control-and-Real-Time-Odometry-in-ROS.git
   cd Autonomous-Mobile-Robot-Control-and-Real-Time-Odometry-in-ROS
