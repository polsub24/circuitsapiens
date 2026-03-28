# 🚗 Multi-Sensor Cooperative Collision Avoidance System using V2V Communication

A decentralized autonomous vehicle system enabling **real-time vehicle-to-vehicle (V2V) communication**, **multi-sensor fusion**, and **cooperative collision avoidance**, developed during **Makeathon 2026**.

🔗 Repository: https://github.com/polsub24/circuitsapiens

---

## 📌 Overview

This project explores how autonomous vehicles can transition from isolated decision-making to a **cooperative multi-agent system**.

Each vehicle:
- Perceives its environment using onboard sensors  
- Shares its state with neighboring vehicles  
- Adapts behavior based on both local sensing and received data  

This mirrors real-world systems used in:
- Autonomous mobility  
- Industrial AGVs  
- Intelligent transportation systems  

---

## ⚙️ System Architecture

### Core Components
- **ESP32** – Processing and communication  
- **VL53L0X ToF Sensor** – Front distance measurement  
- **Ultrasonic Sensors** – Side obstacle detection  
- **ESP-NOW Protocol** – Low-latency V2V communication  
- **GPS Module** – Location awareness and scalability  
- **OLED Display** – Real-time telemetry and diagnostics  

---

## 🔄 Working Principle

1. **Sensing Layer**
   - Front and side obstacle detection using sensor fusion  

2. **Communication Layer**
   - Vehicles exchange:
     - Distance information  
     - Motion state (stop / move / obstacle)  

3. **Decision Layer**
   - Combines local sensor data with received vehicle data  
   - Executes cooperative collision avoidance and navigation  

4. **Visualization Layer**
   - OLED provides live system-level insights  

---

## 🚀 Features

- Multi-sensor fusion for improved reliability  
- Real-time V2V communication using ESP-NOW  
- Cooperative collision avoidance between vehicles  
- Fault-tolerant handling of sensor noise and packet loss  
- Live telemetry for debugging and monitoring  
- Modular and scalable architecture  

---

## 🧠 Key Innovation

The system introduces a shift from **reactive, isolated control** to **cooperative intelligence**, where vehicles:
- Share perception  
- Influence each other's decisions  
- Operate as a distributed network  

---

## 📈 Scalability & Applications

This architecture can be extended to:

- Multi-vehicle convoy or swarm systems  
- Industrial AGVs and warehouse automation  
- Integration with V2X infrastructure (traffic signals, roadside units)  
- Transition to DSRC / C-V2X for real-world deployment  
- Smart mobility and connected transportation ecosystems  

---

## 🏆 Achievement

The project progressed through multiple evaluation stages and was placed among the **Top 10 teams at Makeathon 2026**.

---

## 📊 System Output

Each vehicle provides real-time feedback including:
- Distance measurements  
- Communication status (TX/RX)  
- Signal strength (RSSI)  
- Current action (movement / stop / turn / follow)  

---

## 👥 Team

- Subham Sarkar  
- [Teammate 2 Name]  
- [Teammate 3 Name]  

---

## 💡 Future Work

- Smooth convoy control using PID-based distance regulation  
- AI-based predictive collision avoidance  
- Cloud/edge integration for fleet-level intelligence  
- Real-time mapping and localization  

---

## 📜 License

This project is open-source and available under the MIT License.
