# Crazyflie-GUI-python

This repository is a **GUI-based control interface** for the [NCKU-Quadrotor-Navigation](https://github.com/Lee-Chun-Yi/NCKU-Quadrotor-Navigation) project.
It serves as a dedicated branch focusing on an **operator-friendly interface** to manage quadrotor control, replacing the need for direct interaction with long and complex Python scripts.

In our previous workflow, quadrotor control relied on a combination of **MATLAB** (for high-level control logic) and **Python** (for low-level command transmission).
However, as the Python scripts grew in complexity, maintaining a stable command transmission frequency became challenging—especially when a single `.py` file handled multiple critical tasks simultaneously.

To address this, we developed a **custom GUI** inspired by [Bitcraze’s cfclient](https://github.com/bitcraze/crazyflie-clients-python), optimized for our **project’s specific needs**.

---

## Key Features

* **Modular Control Architecture**
  Separates communication, logging, and control functions into independent threads to maintain stable transmission rates.

* **Real-Time Parameter Monitoring**
  Displays key telemetry such as battery voltage, position, and attitude directly in the GUI.

* **Custom Command Panels**
  Enables direct RPYT or PWM output to the quadrotor, compatible with both **2PID** and **4PID** control architectures.

* **Logging & Diagnostics**
  Supports real-time logging of Vicon 6DoF data, control commands, and system performance metrics.

---

## Project Structure

* `src/`
  Main GUI source code, organized into modules for connection handling, flight control, and logging.

* `ui/`
  Tkinter-based interface definitions and style configuration.

* `docs/`
  User guides, setup instructions, and troubleshooting reference.

---


## Ongoing Plans

* Integration with **MATLAB/Simulink** live control
* Enhanced logging visualization (real-time plotting within the GUI)
* Safety features such as obstacle proximity alerts and emergency override

## Project Timeline – Craztflie GUI Python

### 2025-05-20 — 完成python control code 與Matlab以及Crazyflie對接後，設計初版GUI

![](https://github.com/Lee-Chun-Yi/crazyflie-GUI-python/blob/main/image/%E8%9E%A2%E5%B9%95%E6%93%B7%E5%8F%96%E7%95%AB%E9%9D%A2%202025-08-11%20005757.png)


### 2025-08-11 — 設計第二版GUI，將原有單一py拆成數個骨架，用以方便後續升級與維修

