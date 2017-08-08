####################################################
## First we have some standard ROS stuff in the header.

# Standard metadata for higher-level flow data types
# sequence ID: consecutively increasing ID 
# Two-integer timestamp
# frameid is the robot name
Header header


# Battery level. This should be a value between 0.0 and 100.0
float32 batteryLevel

# For debugging, the last velocity command received:
time lastCommandTime
geometry_msgs/Twist lastCommand

# Time stamp of the last valid response received from vehicle side, ie udp status msg from roburoc6 msrs
time lastVehicleResponseTime

# Is the e-stop pressed?
bool isEStopPressed

# Are the brakes on?
bool areBrakesOn

# Is the robot in an error state?
bool isRobotInErrorState

# General health message that provides a human-readable high level status of the robot.
string robotStatus
