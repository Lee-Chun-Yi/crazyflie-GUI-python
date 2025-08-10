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

## Dependencies

* **Python ≥ 3.10**
* [`cflib`](https://github.com/bitcraze/crazyflie-lib-python) for Crazyflie communication
* **Tkinter** for GUI rendering
* **Zadig** (Windows only) for Crazyradio PA driver installation

---

## Future Plans

* Integration with **MATLAB/Simulink** live control
* Enhanced logging visualization (real-time plotting within the GUI)
* Safety features such as obstacle proximity alerts and emergency override


