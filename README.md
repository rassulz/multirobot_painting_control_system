# Multirobot Painting Control System

A distributed control framework for a multirobot industrial painting system using MATLAB, PLC-based automation, KUKA industrial robot control, and RoArm-M2-S robotic manipulators.

This project was developed as a diploma/research project focused on the integration of industrial automation and robotics for a smart painting line. The system combines high-level coordination, robot motion control, PLC logic, and visualization tools into one experimental automation framework.

---

## Project Overview

The main idea of this project is to design and test a multirobot painting system where different robotic and automation components work together in a coordinated industrial process.

The system includes:

- **KUKA KR 10 R1100-2** industrial robot for pick-and-place operations
- **RoArm-M2-S** robotic manipulators for painting-related motion
- **MATLAB** for robot communication, trajectory generation, GUI control, and telemetry
- **PLC / TIA Portal** projects for industrial control logic and visualization
- **Network-based communication** using TCP/IP, HTTP/JSON, and robot-specific interfaces

The project demonstrates how industrial robots, lightweight robotic arms, PLC logic, and MATLAB-based supervisory control can be integrated into a distributed automation system.

---

## System Concept

The proposed system represents an automated painting cell with the following general workflow:

1. An object is transported by a conveyor system.
2. The PLC controls the basic sequence, interlocks, and process states.
3. MATLAB acts as a coordination and monitoring environment.
4. The KUKA robot performs pick-and-place operations.
5. The RoArm manipulators perform painting-related movements.
6. The system state can be monitored through MATLAB GUI and PLC/HMI visualization.

The project is not only focused on robot movement, but also on the interaction between robotics, industrial control, and communication layers.

---

## Key Features

- MATLAB-based robot control
- KUKA robot communication through TCP/IP and KukaVarProxy
- RoArm-M2-S control through WiFi/HTTP and JSON commands
- Dual-arm painting process GUI
- Pick-and-place trajectory generation
- Smooth motion execution using trajectory planning
- Real-time telemetry and event logging
- PLC/TIA Portal project archives
- Industrial automation-oriented project structure

---

## Technologies Used

| Area | Tools / Technologies |
|---|---|
| Robot control | MATLAB |
| Industrial robot | KUKA KR 10 R1100-2 |
| Robot controller | KUKA KRC4 Compact |
| Lightweight manipulators | Waveshare RoArm-M2-S |
| PLC development | Siemens TIA Portal |
| PLC platform | SIMATIC PLC |
| Communication | TCP/IP, HTTP, JSON, WiFi |
| KUKA interface | KukaVarProxy |
| Visualization | MATLAB GUI, PLC/HMI visualization |

---

## Repository Structure

```text
multirobot_painting_control_system/
│
├── Hardware/
│   └── ReadMe.md
│
├── MATLAB/
│   ├── KUKA_control/
│   │   ├── KukaTest_Connection.m
│   │   ├── create_pick_and_place_trajectory.m
│   │   ├── excecute_pick_and_place_trajectory.m
│   │   ├── kuka_pick_and_place_gui_vacuum.m
│   │   ├── MatLabControl.dat
│   │   ├── MatlabControl.src
│   │   └── README.md
│   │
│   ├── Roarm_control/
│   │   ├── RoarmM2_MotionControl_v2.m
│   │   ├── RoarmM2_CompleteExample.m
│   │   ├── RoarmM2_WiFi_Demo.m
│   │   ├── Roarm_config.m
│   │   ├── go_to_point.m
│   │   ├── roarms_painting_process_gui.m
│   │   └── README.m
│   │
│   └── README.md
│
├── PLC/
│   ├── PID_Updated.rar
│   ├── PLC_code_Visualization.rar
│   ├── Tia_portal_Almat_24_12_2025.rar
│   └── README.md
│
├── docs/
│   └── README.md
│
├── diagnose_wrist_results.mat
│
└── README.md
