# Crazyflie-GUI-python

We named this GUI as: cfmarslab.

This repository is a **GUI-based control interface** for the [NCKU-Quadrotor-Navigation](https://github.com/Lee-Chun-Yi/NCKU-Quadrotor-Navigation) project.
It serves as a dedicated branch focusing on an operator-friendly interface to manage quadrotor control, replacing the need for direct interaction with long and complex Python scripts.

In our previous workflow, quadrotor control relied on a combination of **MATLAB**  and **Python** .
However, as the Python scripts grew in complexity, maintaining a stable command transmission frequency became challenging—especially when a single `.py` file handled multiple critical tasks simultaneously.

To address this, we developed a **custom GUI** inspired by [Bitcraze’s cfclient](https://github.com/bitcraze/crazyflie-clients-python), optimized for our project’s specific needs.



## Project Structure

* `src/`
  Main GUI source code, organized into modules for connection handling, flight control, and logging.


* `docs/`
  User guides, setup instructions, and troubleshooting reference.

---


## Project Timeline – Crazyflie GUI Python

### **2025-05-13 — GUI Project Start**

* Began development of the GUI project
---

### **2025-05-20 — Our First GUI**

* Completed integration between Python control code, MATLAB, and Crazyflie
* Designed first GUI version for operator-friendly control
  ![](https://github.com/Lee-Chun-Yi/crazyflie-GUI-python/blob/main/image/%E8%9E%A2%E5%B9%95%E6%93%B7%E5%8F%96%E7%95%AB%E9%9D%A2%202025-08-11%20005757.png)

---

### **2025-08-11 — Second GUI Version**

* Refactored original single `.py` into **multiple modular files** for easier upgrades and maintenance
* Improved GUI layout for better usability
  ![](https://github.com/Lee-Chun-Yi/crazyflie-GUI-python/blob/main/image/%E8%9E%A2%E5%B9%95%E6%93%B7%E5%8F%96%E7%95%AB%E9%9D%A2%202025-08-11%20010041.png)
