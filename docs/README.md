# Documentation Assets

Images, renders, diagrams, and screenshots used across the repository's README files, plus the full project documents. Sourced from the project's CAD work, 3D renders, software screenshots, and the diploma presentation.

## Documents

| File | Description |
|---|---|
| `Diploma_Final.pdf` | Full graduation thesis (KBTU, 2026). |
| `Diploma_presentation.pdf` | Defense presentation slides. |

## `images/`

### Renders & process
| File | Description |
|---|---|
| `render_industrial_line.png` | 3D render of the full industrial painting line. |
| `render_industrial_scale.png` | Industrial-scale process render (captioned). |
| `render_laboratory_cell.png` | 3D render of the laboratory workstation. |
| `render_kuka_conveyor.png` | KUKA picking a part from the conveyor (render). |
| `render_roarm_spraygun.png` | RoArm-M2-S with the spray-gun end-effector (render). |
| `render_magnetic_endeffector.png` | Electromagnetic gripper close-up (render). |
| `process_stages.png` | Painting → drying → pick-and-place stages. |
| `process_flowchart.png` | Full process / control flowchart. |

### Architecture & schemes
| File | Description |
|---|---|
| `architecture_three_level.png` | Three-level control architecture (Level 0/1/2). |
| `architecture_block_diagram.png` | Network topology — PROFINET, OPC UA, TCP/IP. |

### Engineering drawings
| File | Description |
|---|---|
| `cad_assembly.jpg` | General assembly with parts list. |
| `cad_kuka_model.jpg` / `cad_roarm_model.jpg` | KUKA / RoArm mechanical drawings. |
| `cad_conveyor.jpg` | Conveyor drawing. |
| `cad_endeffector_exploded.jpg` | Exploded view of the spray-gun end-effector. |
| `kinematics_kuka.jpg` / `kinematics_roarm.jpg` | Kinematic diagrams. |
| `workarea_kuka.jpg` / `workarea_roarm.jpg` | Working-envelope drawings. |
| `electrical_roarm.jpg` | RoArm-M2-S electrical / ESP32 wiring scheme. |

### Software screenshots
| File | Description |
|---|---|
| `gui_kuka_pickplace.jpg` | KUKA pick-and-place MATLAB GUI. |
| `gui_roarm_painting.jpg` | RoArm dual-arm painting MATLAB GUI. |
| `gui_kuka_picture.png` / `3d_preview_kuka.png` | Earlier KUKA GUI / 3D trajectory preview. |
| `hmi_overview.png` / `hmi_control_panel.jpg` | WinCC HMI screens. |
| `hmi_pid_trend.png` | WinCC PID trend / tuning screen. |
| `tia_watchtable_opcua.png` | TIA Portal watch table + OPC UA interface. |
| `simulink_supervisory.png` / `simulink_opcua_inputs.png` | Simulink supervisory model. |

### Hardware
| File | Description |
|---|---|
| `Kuka_rend.jpg` / `Roarm_render.jpg` | Robot renders. |
| `hardware_roarm.png` | RoArm-M2-S product photo. |
| `hardware_bldc_motor.png` | 57BLF01 conveyor BLDC motor. |

When adding new images, place them here and reference them as `docs/images/<file>` so the links resolve on GitHub.
