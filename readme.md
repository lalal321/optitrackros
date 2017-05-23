Add the folder to the src directory of a catkin workspace and perform catkin_make.


Then run using "rosrun optitrackros forward_pos_to_drone.py"


The script launches a topic called "CameraStream" and publishes 
the following data in a message type defined in "Camera.msg"


float32 label
float32[3] pos
float32[4] quart
float32 timestamp


The label corresponds to the label value defined in the dictionary "bodynames" on line 37 of "forward_pos_to_drone.py".

The keys in the dictionary must correspond to the names of the rigid bodies that are to be tracked as defined on the Motive software for Optitrack.





