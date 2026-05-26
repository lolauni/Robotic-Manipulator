# Robotic Manipulator - Control Architecture (UC3M)

This repository contains the code, design, and documentation for the development of a functional **robotic manipulator arm**, designed and implemented as a final project for the *Robot Control Architecture* course at **Universidad Carlos III de Madrid (UC3M)**.

The project covers the entire development cycle, from mechanical design and 3D printing manufacturing to the implementation of an electronic closed-loop control system using DC motors and PID algorithms.

---

## 🚀 Key Features

* **Design & Manufacturing:** Entire mechanical structure modeled and fabricated via **3D Printing**.
* **Efficient Actuation:** DC motors equipped with encoders for real-time position feedback.
* **Control System:** Implementation of a **PID (Proportional, Integral, and Derivative) controller** to ensure smooth, precise movements and correct position errors on the fly.
* **Software Architecture:** Modular code structured for sensor reading, high-frequency control loop calculations, and motor actuation via PWM signals.

---

## 🛠️ System Architecture (Hardware & Electronics)

The system is built upon the following main blocks:

1. **Mechanical Structure:** 3D printed components optimized to withstand motor torque and the weight of the links.
2. **Actuators and Sensors:** * DC gearmotors.
   * Integrated magnetic/optical encoders to measure the angular position of the joints.
3. **Power Stage:** Motor drivers (H-Bridge) to control rotation direction and speed using PWM signals.
4. **Control Unit:** Microcontroller responsible for running the PID control loop at high frequencies.

---

## 💻 Software Structure

The main codebase is located inside the [`todo_junto_final`](./todo_junto_final) folder. The control algorithm follows this pipeline:

* **Encoder Reading:** Pulse capture using hardware interrupts to calculate the current position of each joint.
* **Error Calculation:** Computing the difference between the desired target (SetPoint) and the actual position.
* **PID Algorithm:** * Proportional Term ($K_p$): Instantaneous error correction.
  * Integral Term ($K_i$): Elimination of steady-state error caused by friction and gravity.
  * Derivative Term ($K_d$): Damping to prevent overshoot and oscillations.
* **PWM Output:** Generation of the control signal fed into the motor drivers.

---

## 📊 Results and Calibration

To achieve manipulator stability, a fine-tuning process was carried out for the PID constants ($K_p, K_i, K_d$), minimizing overshoot and settling time for each joint under step inputs and continuous trajectories.

---

## 📸 Gallery & Demonstration

| Robotic Arm View | 
| :---: | (<img width="632" height="606" alt="image" src="https://github.com/user-attachments/assets/4d2f8be0-8797-455d-a77a-2fd176c26b02" />
) |



---

Developed by:
* **Lola Serrano** - [@lolauni](https://github.com/lolauni)
* *(Add your group mates here if applicable)*
