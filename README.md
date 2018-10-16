# irl_robots

Collection of custom ROS drivers for robots within the interactive robotics lab.

This code suits our purpose and due to limited time on our side, feature requests will most likely be ignored. 
On the other hand, feel free to add some features on your own and make a pull request. 

Please cite us when using this code in your project. 

# Usage

The following gives a brief overview of how this node and the different robots can be used. 

# r2fg

Robotiq 2 Finger Gripper:
This requires a ROS-Parameter server with the following properties: 
 - r2fg/port : Set this to the corresponding USB-Port (e.g. /dev/ttyUSB0)

Available messages:
 - gSimpleControl: Simple message for quick control. This just takes Speed/Force/Position and sets all other values to active:true, go:true, ...
 - r2fgControl: This message gives you the full functionality of the gripper. See the Robotiq documentation for a description of all the values (http://support.robotiq.com/pages/viewpage.action?pageId=8520161).
 - r2fgStatus: Contains all status information given by the gripper. See the Robotiq documentation for a description of all the values (http://support.robotiq.com/pages/viewpage.action?pageId=8520162).

Topics: 
 - /r2fg/control : Expects a r2fgControl message
 - /r2fg/simplecontrol : Expects a gSimpleControl message
 - /r2fg/status : Publishes a r2fgStatus message at 30 Hz

# r3fg

Robotiq 3 Finger Gripper:
This requires a ROS-Parameter server with the following properties: 
 - r3fg/ip : IP of the gripper. Defaults by Robotiq is 192.168.1.11
 - r3fg/port : The port of the MODBUS-TCP protocol. This is usually 502
 - r3fg/refreshrate : The rate in which the status is published in Hz. Do not exceed 100 Hz. 

Available messages:
 - gSimpleControl: Simple message for quick control. This just takes Speed/Force/Position and sets all other values to active:true, go:true, ...
 - r3fgControl: This message gives you the full functionality of the gripper. See the Robotiq documentation for a description of all the values (http://support.robotiq.com/pages/viewpage.action?pageId=590044).
 - r3fgFingerControl : Used in r3fgControl
 - r3fgStatus: Contains all status information given by the gripper. See the Robotiq documentation for a description of all the values (http://support.robotiq.com/pages/viewpage.action?pageId=590045).
 - r3fgFingerStatus : Used in r3fgStatus

Topics: 
 - /r3fg/control : Expects a r3fgControl message
 - /r3fg/simplecontrol : Expects a gSimpleControl message
 - /r3fg/status : Publishes a r3fgStatus message

# ur5

Controller for the UR5. Do not use this directly if you want to have a continuous high frequency controller for the UR5. This is only intended for single move commands. 
This nodes requires the following parameters to be available through the ROS-Parameterserver
  - ur5/ip : The IP of the UR5
  - ur5/port : The port of the Real-Time-Interface. This should be 30003

Please not that we did not made all properties available through this node. We limited it to things we explicitly needed to control the robot. Available messages:
 - ur5Tool : Publishes the tool state of the UR5 at 125 Hz. It contains the position, rotation and velocities of the ToolCenterPoint 
 - ur5Joints : Publishes the current joint angles in radiant as well as the current joint velocities
 - ur5Status : Publishes the current status of the UR5, representing its mode: Idle, Booting, EmergencyStop, ... . See the UR5 documentation for details
 - ur5Control : Used to control the UR5. See the documentation for URScript: http://www.sysaxes.com/manuels/scriptmanual_en_3.1.pdf 
   - command : URScript language command. Currently supports movel, movej, speedj, servoj
   - values : The six values -> Either six joint angles or TCP 3x position and 3x rotation
   - acceleration : Property of some commands
   - velocity : Property of some commands
   - time : Property of some commands
   - blend : Property of some commands
   - gain : Property of some commands
   - lookahead : Property of some commands
   - jointcontrol : Set to true if values contain jointangles, else set to false

Topics: 
 - /ur5/control : Expects a ur5Control message
 - /ur5/joints : Publishes the joint status at 125Hz
 - /ur5/tool : Publishes the tool status at 125Hz
 - /ur5/status : Publishes the robots status at 125Hz

Additionally there is a service to control the UR5. It is designed to use time commands and will block until the UR5 is done. Use movel or movej in this service. The service is used under the '/ur5/ur5SrvControl' name. 

# ur5_continuous_controller

This is the continuous controller for the UR5. It is able to control the UR5 with a high frequency without any interrupts regarding its movement. Basically this is a PID controller on top of the ur5 node. You can set its corresponding values in the ur5network.cpp file of the ur5 code (TODO: Put that in a config file...). 
Currently it expects a matrix of the form row by column 

Messages:
 - matrix : Represents a matrix. Rows are the joint angles and columns are the number of control-points given to the controller in this command. There should always be six rows containing the target joint angles in radians. Row[0] contains the angle for the base joint and row[5] for the last joint at the TCP. The number of columns may vary depending on your particular usage. 
 - row : Contained in the matrix message

Topics:
 - /ur5/continuous_controller : Expects a matrix as input