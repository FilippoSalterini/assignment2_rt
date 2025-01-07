# Assignment2_ROS1 - Research Track 1
## Overview
This assignment requires to implements 2 nodes such that:
1. **Node1**: A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the 
feedback/status of the action server to know when the target has been reached. The node also publishes the
robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
topic /odom;
2. **Node2_service**: A service node that, when called, returnsthe coordinates of the last target sent by the user;

## Nodes

### NODE1 node description
* File name: Node.cpp
* Key Features: 
  

### DISTANCE node description
A node that checks the relative distance between turtle1 and turtle2, publish on a topic the distance, stops the moving turtle if the two turtles are “too close” and also stops the moving turtle if the position is too close to the boundaries
* File name:
* Key Features:
Declare 2 variables name ```turtle1_position``` and ```turtle2_position``` of the message type ```turtlesim::Pose``` that contains position and orientation data.  
Defined some function :


Inside the main :  


## How to run it
1. 
2.
3. 
4. T

## Notes






