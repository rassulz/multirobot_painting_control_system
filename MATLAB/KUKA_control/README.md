# KUKA KR 10 R1100-2 Access and Control via WiFi and Using MatLab. 
Kuka KR R 1100 is the agilus series and uses the KRC4 Compact controller. The KUKA is already in the local network and we can access it and control it via MatLAB using the WiFi connection. The simplest and the best method of performing this can be through the use of the KukaVarProcy which is a free tool that can run in the robot controller. This allows MatLab to read and write global variables on the robot. You also need the KRL (kuka Robot Language) program to react to those variables such as moving to the new position. 

## Prerequisites. 
1. Administrative Access to the KUKA smartPAD wit expert mode enabled. Use the "Stupid" password if the administrative privileges are locked. 
2. USB Stick to transfer files to the robot controller. 
3. MatLAB r2025b. Ensure that the MatLab has the Instrument Control Toolbox for TCP/IP. 
4. Ensure that the settings for the robot they are in T1 Mode which is the slow speed mode first. 
5. Ensure that no firewalls blocks port 7000 on the robot controller or your PC. 
6. Download the KukaVarProxy.exe from Github using the following links: https://github.com/ImtsSrl/KUKAVARPROXY (clone the repo or grab the executable from releases if available; build it if needed using VB6 as per repo instructions).


## Step 1: Network Configuration on the KUKA Controller
Ensure that the controller's Ethernet interface (KLI, virtual network  5) is in the SAME SUBNET as your router. For instance if the default IP is 172.31.1.147 then it means that this might not match your homenetwork which is 192.168.x.x. 

1. Power on the robot controller and connect the smartPAD. 
2. Log in as expert: Press the key icon on the smartPAD, and enter the credentials. 
3. Navigate to Menu>Start-up>Network configuration. 
4. Select the interface labeled Virtual5 (This is the KLI for external communication)
5. If using DHCP (recommended if your router assigns the IPs automatically)
This can be performed by enabling the DHCP and rebooting the controller (Menu>Shutdown>Reboot)
6. If setting a static IP (if DHCP fails or for stability):
Disable DHCP.
Set IP address (e.g., 192.168.1.10 if your router is 192.168.1.1).
Set subnet mask (usually 255.255.255.0).
Set gateway to your router's IP (e.g., 192.168.1.1).
Apply and reboot the controller.

7. Note the assigned IP address (visible in the network configuration screen after reboot).
8. If the KUKA and the router has IP addresses already configured then skip step 5, 6 and 7. 

## Step 2: Connect your PC to the Network via WiFI and Verify Connectivity.
1. Connect your PC to the same router's WiFi network. 
2. Open the command prompt and ping the IP address to ensure that there is connectivity. ping <robot_IP>
If successful, you see replies. If not, check cables, firewall, or IP mismatch. Disable PC firewall temporarily if needed.

3.  If pinging works, the robot is accessible over WiFi.

## Step 3: Installation and  Running the KukaVarProxy on the Robot Controller. 

1. Copy KukavarProxy.exe to a USB stick.
2. Insert the USB into the smartPAD or controller (USB port on the front).
3. On the smartPAD, navigate to Menu > File browser, copy the .exe to the desktop or C:\KRC\USER.
4. Double-tap the .exe on the controller's Windows interface (accessible via smartPAD or VNC if set up) to launch it. It should run minimized and listen on port 7000.
5. Verify it's running: On your PC, use a tool like telnet or netcat to connect to <robot_IP>:7000. If it connects, it's active.

## Step 4: Set up the KRL program for robot control
This is supposed to be set on the SmartPAD so that it can monitor the variables set by MatLAB and excecute the movements. 
1. On the SmartPAD, create the new module in the expert mode. 

   Go to menu > Program > New
   Name it MatLabControl.src and MatLabControl.dat

The content for the files can be found in the MatLabControl.src and MatLabControl.dat

4. Select and run the program in T1 mode (turn key to T1, press green button). It will wait for variable changes.

Safety Note: Always enable the deadman switch on the smartPAD. Start with slow velocities. Ensure the target positions are within the robot's workspace to avoid collisions.

## Step 5: Control the Robot from MATLAB
Use MATLAB's tcpclient to connect and write variables using the KukaVarProxy protocol. All values are sent as big-endian uint16 (for lengths/IDs) and ASCII strings.
Run the script. It connects, writes the new position to target_pos, sets move_trigger to TRUE, and the robot program executes the move.
For reading a variable (e.g., current position $POS_ACT):
Modify the writeVar function for mode 0 (read): Remove value parts, adjust contentLen.
Parse the response value from the answer message.

## Troubleshooting

No connection: Check IP, port 7000 open (add exception in Windows Firewall on controller: Control Panel > Firewall > Allow app > Select KukavarProxy.exe).
Variable errors: Ensure variables are global and declared correctly. Use full name if in a module (e.g., $config.dat:target_pos).
Movement issues: Run in T1 mode first. If errors, check smartPAD for messages.
File transfer issues: If USB fails, set up Windows sharing on the controller (Menu > Start-up > User > Add user with share permissions).
If KukaVarProxy won't run: Ensure controller Windows is compatible (KRC4 uses Windows Embedded); rebuild from source if needed.

