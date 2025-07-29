# 🐾 DOGBot - 3D Printed Quadruped Robot Dog

[![Repository](https://img.shields.io/badge/GitHub-DOGBot--3D--Printed--Quadruped--Robot--Dog-blue?logo=github)](https://github.com/Shobhit3244/DOGBot-3D-Printed-Quadruped-Robot-Dog)

> Open-source quadruped robot with real-time inverse kinematics, customizable gait cycles, precision servo calibration, and full Python control on Raspberry Pi.

---

## Overview

**DOGBot - 3D Printed Quadruped Robot Dog** is a modular and fully programmable quadruped robotics platform designed for robotics enthusiasts, educators, and researchers interested in motion control and walking algorithms.

- **Inverse kinematics-powered:** Real-time joint angle computation from foot endpoint targets for smooth and coordinated leg movement.
- **Fully parameterized gait:** Customize step size, shape, and style — including elliptical foot trajectories.
- **Ultra-configurable:** Easily configure hardware segment lengths, servo channel mappings, and joint limits using YAML.
- **Skill-focused:** Demonstrates servo electronics, kinematics, Python programming, and open-source collaboration, excellent for learning and prototyping.
- **Built for practical use:** Includes a CLI servo calibrator, robust code organization, and safeguards against servo damage.

---

## 🚀 Features

- **Inverse Kinematics Solver:** Real-time, parametric IK for each leg to calculate precise joint angles.
- **Customizable Gaits:** Walk, turn, and hop with configurable trajectories, including elliptical paths.
- **Calibration Mode:** Interactive servo calibration tool for setting min/mid/max angle limits per servo.
- **YAML-Based Configurations:** All key parameters stored in human-readable config files for easy tweaking.
- **PWM Servo Control:** Controls up to 12 hobby servos via PCA9685 PWM controller on Raspberry Pi 4.
- **Modular Python Codebase:** Well-structured for extension, including ROS compatibility and sensor integration.
- **GitHub Version Control:** Sync your code easily, enabling collaborative development and easy updates.

---

## 📦 Quickstart

1. **Clone the repository on your Raspberry Pi:**

    ```
    git clone https://github.com/Shobhit3244/DOGBot-3D-Printed-Quadruped-Robot-Dog.git
    cd DOGBot-3D-Printed-Quadruped-Robot-Dog
    ```

2. **Set up Python virtual environment and install requirements:**

    ```
    sudo apt update
    sudo apt install python3-venv python3-pip i2c-tools -y
    python3 -m venv ~/dogbot-venv
    source ~/dogbot-venv/bin/activate
    pip install -r requirements.txt
    ```

3. **Configure your robot parameters and calibrate servos:**

    - Edit `configs/servo_map.yaml` and `configs/limb_lengths.yaml` as needed.
    - Run the calibration tool to set servo limits:

      ```
      cd src/servo_control
      python servo_configurator.py
      ```

4. **Run the main gait controller:**

    ```
    cd ../
    python main.py
    ```

---

## 📂 Project Structure
```
dogbot_ws/
├── configs
│   ├── limb_lengths.yaml      # Robot limb segment lengths (cm)
│   ├── servo_calib.yaml       # Servo min, mid, max calibration (degrees)
│   └── servo_map.yaml         # PCA9685 channel ↔ joint names mapping
└── src
    ├── gait
    │   ├── gait.py            # Gait pattern generator
    │   ├── __init__.py        # Package initialization
    │   └── __pycache__        # Compiled Python files
    │       ├── gait.cpython-312.pyc
    │       └── __init__.cpython-312.pyc
    ├── ik
    │   ├── ik.py              # Inverse kinematics module
    │   ├── __init__.py        # Package initialization
    │   └── __pycache__        # Compiled Python files
    │       ├── ik.cpython-312.pyc
    │       └── __init__.cpython-312.pyc
    ├── main.py                # Main program managing gait cycles and control
    └── servo_control
        ├── __init__.py        # Package initialization
        ├── servo_configurator.py  # Interactive servo calibration tool
        ├── servo_control.py   # Servo PWM control module
        └── __pycache__        # Compiled Python files
            ├── __init__.cpython-312.pyc
            └── servo_control.cpython-312.pyc


```
---

## 💡 Highlights

- **Designed for Makers:** Fully 3D printable frame, hobby servo-compatible electronics.
- **Educational Focus:** Deep insight into quadruped locomotion, inverse kinematics, and servo control.
- **Safety First:** Calibrate servo ranges before operation to protect your servos.
- **Open and Modular:** Easily add sensors, cameras, onboard feedback, or ROS nodes.

---

## 🛠️ Technologies Used

- **Hardware:** Raspberry Pi 4, PCA9685 PWM controller, 12 hobby servo motors
- **Software:** Python 3, Numpy, PyYAML, Adafruit CircuitPython Servo libraries
- **Concepts:** Real-time inverse kinematics, gait planning and sequencing, servo calibration, CLI design

---

## 🏃 Typical Behavior Sequence

- Walk forward
- Turn right in a full circle
- Walk forward again
- Turn left in a full circle
- Hop three times
- Pause for five minutes
- Repeat the sequence

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome!  
Feel free to fork the repository and submit pull requests.

---

## 📜 License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for details.

---

## 📞 Contact

Created by [Your Name / Team].  
Feel free to reach out via GitHub or your preferred contact method.

---

Enjoy building and walking your DOGBot! 🐕🤖

