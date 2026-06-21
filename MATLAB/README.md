# MATLAB Supervisory Layer

This directory contains the **MATLAB-based supervisory control layer** of the multirobot painting system. It is the high-level tier of the [three-tier architecture](../README.md): it generates motion trajectories, coordinates the robots within one painting cycle, exchanges safety handshakes with the PLC, and visualizes real-time telemetry.

MATLAB does **not** replace the PLC — the Siemens S7-1500 remains responsible for deterministic sequencing and safety interlocks. Instead, MATLAB adds the flexible, computation-heavy capabilities (trajectory planning, kinematics, multi-robot coordination, monitoring) that are awkward to implement in ladder logic.

## Modules

| Module | Robot | Transport | Purpose |
|---|---|---|---|
| [`KUKA_control/`](KUKA_control/README.md) | KUKA KR10 R1100-2 (6-DOF) | TCP/IP via **KukaVarProxy** (port 7000) | Pick-and-place of finished parts; quintic-smoothstep trajectories streamed at ~83 Hz with live telemetry, joint-limit guards, and a magnetic/vacuum end-effector |
| [`Roarm_control/`](Roarm_control/README.md) | 2× Waveshare RoArm-M2-S (4-DOF) | **HTTP/JSON** over Wi-Fi | Simultaneous spray painting; object-oriented control classes and a dual-arm painting application with reachability checks and telemetry |

## How it fits together

```
            ┌──────────────────────────────────────────┐
            │            MATLAB R2025b host              │
            │                                            │
            │   KUKA_control/        Roarm_control/      │
            │   (pick & place)       (painting)          │
            └───────┬──────────────────────┬─────────────┘
                    │ TCP/IP               │ HTTP/JSON
                    │ KukaVarProxy         │ Wi-Fi
              ┌─────▼─────┐          ┌─────▼─────┐  ┌─────▼─────┐
              │ KUKA KR10 │          │ RoArm #1  │  │ RoArm #2  │
              └───────────┘          └───────────┘  └───────────┘

            (PLC ↔ MATLAB safety handshake over OPC UA, 50 ms)
```

## Prerequisites

- **MATLAB R2025b** (earlier releases work for most scripts; some live scripts target R2021a).
- **Instrument Control Toolbox** — required for `tcpclient` TCP/IP communication with the KUKA controller.
- **Robotics System Toolbox** — used by parts of the KUKA kinematics library and unit tests.
- A network reachable from the MATLAB host to every device (see each module README for IP configuration).

## Quick start

```matlab
% --- KUKA pick-and-place (GUI) ---
cd KUKA_control
kuka_pick_and_place_gui_magnetic        % design poses, preview, execute

% --- RoArm dual-arm painting (GUI) ---
cd ../Roarm_control
roarms_painting_process                 % Ping arms, then START PAINTING
```

See the per-module READMEs for full setup, the communication protocols, and the complete command/API reference.
