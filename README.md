# Parameter-Estimation-and-Waypoint-Navigation
Unmanned Vehicle

To connect 2 Phidgets Motor Controllers with Simulink through ROS in Ubuntu:
Start ROS Master Node
open a new terminal window, and type "roscore"

Launch 1065MotorControl.launch file by "roslaunch phidgets 1065MotorControl.launch"

In case that registration process keeps respawning after running above code,
Make sure that both of the Phidgets Board's serial number is correct in .launch file
and repeat steps from starting

Open Matlab and upload GNC and vehicle parameters into workspace.

Run Simulink Model with desired WayPoints in GNC parameters.
	
