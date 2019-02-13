# Chasing_Ball
Raspberry Pi based robot chases a ball using PID control and combination of various sensory data

PID control and combining multiple sensor inputs (camera and lidar). The goal was to have the robot chase a desired object observed in its localcoordinate frame. Specifically, this meant the robot always faced the object and maintained a desired distance from the object. 

Created a ROS package with three different scripts. First node subscribed to the raspberyy pi camera node and publised the location of the center of the object being tracked. Second node subscribed to receive the location of the object and published the angular position and distance of the object using a combination of the LIDAR data and camera data. In final node, PID control was implemented, One controller acted on the angular error to make the robot face the object while the other acted on the linear error to make the robot maintain a distance from the object. This node published the velocity commands for the robot to follow.
