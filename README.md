# Crazyflie-GUI-python

We named this GUI as: **cfmarslab**.

This repository is a **GUI-based control interface** for the [NCKU-Quadrotor-Navigation](https://github.com/Lee-Chun-Yi/NCKU-Quadrotor-Navigation) project.

This is project is also suit to anyone who is bridging  **MATLAB**  and **Crazyflie 2.1/Bolt** in UAV control, and of course, using **Vicon** as the 6DOF source.


## Key functions

- **Connection & Telemetry**
  - **Connect**, **Disconnect**, and **Scan** buttons with channel, bitrate, and address selectors plus MRU URI history
  - Top-right readouts show latency, RSSI, and VBAT, while a second row reports P95/P99 jitter and deadline miss percentage from the active control loop
    
- **XYZ → MATLAB (UDP 51002)**
  - Enter X, Y, Z targets, choose a **Rate (Hz)**, and **Start** or **Stop** streaming coordinates via UDP 51002; values transmit as little‑endian floats at the selected rate
    
- **2‑PID Controls**
  - **Start/Stop** the RPYT setpoint loop, set **Rate (Hz)**, and monitor the loop’s **Actual** frequency in real time
    
- **4‑PID Controls (PWM)**
  - Switch between **Manual entry** and **UDP 8888**, manage **Start/Stop** and **Rate (Hz)**, and view the loop’s **Actual** rate for feedback
    
- **Safety actions**
  - **Emergency stop** instantly zeros RPYT or motor outputs; **Land (ramp down)** gradually reduces thrust for a controlled descent
    
- **Vicon (UDP 51001)**
  - Start/stop the receiver, monitor live X/Y/Z values, set target **Rate (Hz)**, define X/Y/Z bounds with **Apply**, and use **Show trail (s)** plus **Decimate** to manage plotting load
  - The 3D plot marks the current point and draws a time-limited trail in real time
    
- **3D View & Controls**
  - A Matplotlib 3D scene with tight margins occupies the right pane; vertical **Elevation** and horizontal **Azimuth** sliders rotate the view interactively

### Typical workflow

1. Launch the GUI, set radio channel/bitrate/address, and click **Connect**.  
2. (Optional) Start **Vicon** or begin **XYZ → MATLAB** streaming at the desired rate.  
3. Arm and start either **2‑PID Controls** or **4‑PID Controls (PWM)**, tuning rates as needed.  
4. Observe telemetry, timing metrics, and the 3D view; adjust bounds, trails, or camera angles.  
5. Use **Land** or **Emergency stop**, clear the console if needed, then **Disconnect**.
   
---

## Project Structure

* `src/cfmarslab`

    
  * `ui.py`
    
    Tk GUI, top telemetry, 3D plot, tabs
    
  * `control.py`
    
     SetpointLoop (RPYT), PWMSetpointLoop, UDPInput, PWMUDPReceiver
    
  * `models.py`
    
    shared state/model
    
  * `link.py`
    
     Crazyflie link/commander wrapper
    
  * `vicon.py`
    
    UDP 51001 receiver (x,y,z,rot) for 3D/trail
    
  * `config.py`
    
    app config: rates, safety, persistence

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
