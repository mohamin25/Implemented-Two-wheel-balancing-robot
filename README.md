# Implemented-Two-wheel-balancing-robot
Implemented-Two-wheel-balancing-robot with Arduino and PID controller

# Self-Balancing Robot Project (Bipedal/Two-Wheeled Inverted Pendulum)

This repository contains the final report, code, and documentation for the **Self-Balancing Robot** project, developed as part of the **Introductory Robotics Course** at Amirkabir University of Technology (Tehran Polytechnic).

The project covers the design, modeling, construction, and classic control (PID) of a two-wheeled inverted pendulum system.

## 📝 Project Summary

The core objective of this project was to complete the full development cycle of a self-balancing robot, encompassing:

1.  **Mechanical Design and Construction:** Component selection (motors, sensors, drivers) and chassis fabrication.
2.  **Dynamic Modeling:** Derivation of the robot's equations of motion essential for controller design (Chapter 4 of the report).
3.  **Controller Design:** Implementing and tuning a PID controller for balance and stability maintenance.
4.  **Coding and Implementation:** Writing and deploying the final control code onto the microcontroller.


## ⚙️ Hardware and Components

The primary components used in the robot's construction are detailed in Chapter 3 of the final report:

* **Motors:** (Specific type and specifications are available in the report.)
* **Motor Driver:** (Details on the driver board are in the report.)
* **Sensor:** An IMU (likely **MPU6050** or similar) used for measuring the robot's tilt angle (pitch) and angular velocity.
* **Microcontroller:** The Central Control Unit (details in the report).
* **Chassis:** Designed with a rectangular shape, optimized for stability and motor placement below the main body.

---

## 🧠 Control System Design 

A classic **PID Controller** was selected and implemented to manage the robot's stability:

### PID Control with Enhancements

* **Primary Goal:** To maintain the robot's vertical angle at the setpoint (zero degrees) to ensure balance.
* **Specific Challenges & Solutions Implemented:**
    1.  **Integral Windup:** To prevent instability caused by accumulating error, an **Anti-Windup** mechanism was added to the integral term.
    2.  **Sensor Calibration:** An automatic calibration routine was utilized to minimize measurement errors from the IMU sensor.
    3.  **Motor Backlash:** Extensive testing identified approximately **10 degrees of motor backlash**. This was compensated for in the code by adding a **10-degree deadband** to significantly improve control during direction changes.
    4.  **Physical Dimensions:** Initial stability issues were resolved by adjusting the robot's physical dimensions from a square to a more stable rectangular shape.

---

## 💻 Code and Documentation

### Repository Contents

* `code/`: Contains the microcontroller code for the robot.
    * `controller.ino` (or similar): The main control file, including sensor reading, filtering, PID logic, and motor commands.
* `docs/`: Contains the project documentation.
    * `final_report.pdf`: The complete project report (in Persian), covering dynamics, hardware design, code structure, and troubleshooting.

---

## 🔗 Getting Started

1.  Review the `final_report.pdf` for a detailed understanding of the dynamic model and control logic.
2.  Inspect the `code/` directory to see the PID implementation and hardware communication protocol.
3.  Configure your microcontroller IDE (e.g., Arduino IDE) with the necessary libraries (e.g., IMU library).
4.  Upload the code to replicate the balancing functionality.

---

<img src="20250115_031611.jpg" alt="ربات تعادلی در حال حرکت" width="400"/>
