# A-Multi-Robot-Platform-for-Mobile-Robots-with-ROS
 Implementation of a fleet of three robots to localize cooperatively a maximal number of flags to win the game. (ROS) 
The mission requires a fleet of at least three robots to localize cooperatively a maximal number of flags in order to win the game. We characterize the robots as follow:
● Each robot has its own ID
● Each robot can communicate with another one
● Each robot embeds an ultrasonic sensor to observe the environment through a restricted range (3 meters)
● Each robot moves in the environment with two wheels
The flags are placed randomly in the robot environment and send a continuous signal. Each flag has its own ID. By calling a service, robots can have an estimation of the distance with the closest flag and the flag’s ID
