# General
Put units in variable name.
500hz -> maybe lower?

# Controllers
- Change the setpoints -> use a trapezoidal velocity profile function
-> use distance, current speed, and velocity profile.
nicht bremsen

# Robots
- Create a wheelchair robot (PRIORITY)
- Bring up G1 Human Robot
- use a Abstracted away Navigation_State reader (PRIORITY)

# Waypoints
- Easier way to create and manage the waypoints
- Global Path planner
- Local path planner

# Sim
- In sim show where the robot was and then teleported to after relocalisation from Ublox
- Transistion to 3D environment so that i can see the rotations and the actual world better.
- Show Trajectory that was made while driving.
- Show future of trajectory maybe? 
- switch between 3d and 2d with a hotkey.
- Show PointCloud for like lidar and such.
- Show Path

# Sensors
- use UBLOX IMU for read navigation state (PRIORITY)

# Sockets


-- # V.2 # -- 
 
decouple viewer from project and have the controllers and sensors send information to a 
memory place where viewers and such could use it.
