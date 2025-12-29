# RoArm-M2-S Configuration and MATLAB Control via WiFi/HTTP

## Overview
This README provides step-by-step instructions to configure one or more **RoArm-M2-S** robotic arms and control them directly from **MATLAB** using WiFi and HTTP-based commands. The provided MATLAB script demonstrates reading status and controlling joint positions, Cartesian coordinates, and gripper for single or dual arms.

The RoArm-M2-S supports two WiFi modes:  
- **AP Mode** (default): Each arm creates its own hotspot.  
- **STA Mode** (recommended for multi-arm or integrated setups): Arms connect to your existing router/hotspot, allowing all devices (PC + arms) to be on the same network.

For MATLAB integration in a multi-robot painting system, **STA mode is strongly recommended**.

## Hardware Requirements
- RoArm-M2-S robotic arm(s)
- Power supply for each arm
- WiFi router or hotspot (for STA mode)
- Computer running MATLAB (R2019b or later recommended, with `webread` and `weboptions` support)

## Step 1: Power On the RoArm-M2-S
1. Connect the power adapter to the RoArm-M2-S.
2. Turn on the arm using the power switch.
3. Wait ~20 seconds for boot-up.
4. The **OLED display** will show the current WiFi mode and IP address.

## Step 2: Configure WiFi in STA Mode (Recommended)
By default, the arm is in AP mode (SSID: `RoArm-M2`, Password: `12345678`, IP: `192.168.4.1`).

To switch to STA mode and connect to your local network:

1. Connect your computer temporarily to the arm's AP hotspot:  
   - SSID: `RoArm-M2`  
   - Password: `12345678`
2. Open a web browser and go to `http://192.168.4.1`
3. Navigate to **WiFi Settings** → **STA Mode**
4. Select your router/hotspot SSID and enter its password.
5. Save and reboot the arm.
6. After reboot, the OLED display will show the new IP address assigned by your router (e.g., `192.168.0.192`).

Repeat for each RoArm-M2-S arm.  
**Important**: Ensure all arms and your MATLAB computer are connected to the **same network**.

## Step 3: Verify Connectivity
From your computer (on the same network):
1. Open a terminal/command prompt.
2. Ping the arm's IP:
   ```bash
   ping 192.168.0.192



# KUKA Configuration and MATLAB Control via WiFi/HTTP