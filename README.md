# Autonomous Box Handling Robot

## Complete System Overview

An ESP32-based autonomous robot capable of navigating a line-mapped environment, collecting boxes using electromagnets, and delivering them to designated drop-off points. The system integrates multiple sensors and actuators to handle navigation, obstacle avoidance, and precise item handling.

---

## Hardware Integration

- **4 Electromagnets** – One mounted on each face for box pickup/release  
- **4 Time-of-Flight (ToF) Sensors** – One on each face for obstacle detection  
- **5-Line Sensor Array** – For line-based navigation and intersection detection  
- **Mechano Wheel Engagement System** – Enables multidirectional movement when needed  
- **Differential Drive Motors** – For standard movement and line following

---

## Navigation System

- **Dijkstra’s Algorithm** – For shortest-path routing between nodes  
- **PID-Controlled Line Following** – Ensures stable and accurate tracking  
- **Intersection Detection** – Recognizes and handles map intersections  
- **Obstacle Avoidance** – Utilizes ToF sensors to dynamically reroute

---

## Box Handling

- **Sequential Pickup** – Follows a planned route to collect from target nodes  
- **Face Rotation** – Rotates robot to use different electromagnets per pickup  
- **Controlled Drop-Off** – Places each box precisely at specified nodes  
- **Electromagnet Control** – Manages pickup/release timing and face coordination

---

## Mission Control

- **State Machine** – Manages 11 distinct states for mission flow  
- **Progress Tracking** – Monitors collected and delivered boxes  
- **Status Reporting** – Provides live system feedback  
- **Error Handling** – Detects and recovers from common faults

---

## Usage Instructions

1. Upload the code to your **ESP32**
2. Ensure **all hardware components are correctly connected**
3. The robot will:
   - Begin at `START_NODE` (default: `"7A"`)
   - **Collect 4 boxes** from `TARGET_NODES` in order
   - **Deliver** to `DROPOFF_NODES` in order
   - **Disengage mechano wheels** upon completion

---

## Customization Points

- **START_NODE, TARGET_NODES, DROPOFF_NODES** – Update based on your map layout  
- **BOX_PICKUP_TIME** – Set duration for electromagnet activation  
- **PID Constants (Kp, Ki, Kd)** – Tune for optimal line-following performance  
- **ToF Sensor Thresholds** – Adjust for your environment's obstacle distances  
- **Pin Assignments** – Match pin mappings to your specific hardware wiring

---

## 📄 License

This project is open-source and freely available for academic and hobbyist use. For commercial applications, please contact the authors.
