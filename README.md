# Self-Balancing Robot with PD Control and Non-Linear Compensation

This repository contains the source code, simulation models, and parameter tuning scripts for a **Two-Wheeled Self-Balancing Robot**. This project was developed to demonstrate stability using a low-cost microcontroller (Arduino Nano 33 IoT) and standard DC motors by implementing a PD controller augmented with static boost logic for deadzone compensation.

## 📂 Repository Structure

- **/Arduino_Firmware**: C++ source code for the Arduino Nano 33 IoT.
- **/Simulation_MATLAB**: MATLAB/Simulink models used to verify the control logic before deployment.

## 🛠️ Hardware Specifications

| Component | Description |
| :--- | :--- |
| **Microcontroller** | Arduino Nano 33 IoT (Cortex-M0+) |
| **IMU** | LSM6DS3 (Integrated on Arduino) |
| **Motors** | 2x 12V DC Geared Motors (200 RPM) |
| **Driver** | MDD3A Dual Channel Motor Driver |
| **Power** | 5x Li-Ion Cells (18.5V Nominal) |
| **Chassis** | Custom Double-Layer Acrylic |

## ⚙️ Control Strategy

The robot uses a **Proportional-Derivative (PD)** controller. Due to the high gear ratio and inertia of the 200 RPM motors, a standard linear PID was insufficient. The control loop includes:

1.  **Sensor Fusion:** Complementary Filter ($\alpha = 0.98$) combining Accelerometer and Gyroscope data.
2.  **PD Loop:** $K_p = 40.0$, $K_d = 1.0$.
3.  **Non-Linear Boost:** A static PWM injection ($\pm 40$) is applied when error $> 8^\circ$ to overcome static friction.
4.  **Deadzone Compensation:** PWM values between 0-35 are clamped to 35.

## 🚀 Getting Started

### Prerequisites
* Arduino IDE
* MATLAB R2020a or newer (for Simulink models)
* **Library:** `Arduino_LSM6DS3` (Install via Arduino Library Manager)

### Installation
1.  Clone this repository.
2.  Open `Arduino_Firmware/Main_Robot_Code.ino` in Arduino IDE.
3.  Connect the Arduino Nano 33 IoT via USB.
4.  **Important:** Place the robot upright on a flat surface before uploading to allow for initial calibration.

## 📊 Simulation Results

The Simulink model (`Robot_Physics_Model.slx`) validates the plant dynamics using the following derived parameters:
* Total Mass: 1.087 kg
* Motor Constant: 25.0
* Center of Mass height: 0.08 m

## 📝 License

This project is open-source and available under the MIT License.
