# Assignment2_ROS1 - Research Track 1
## Overview
This assignment requires to implements 2 nodes such that:
1. **Node1**: A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the 
feedback/status of the action server to know when the target has been reached. The node also publishes the
robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
topic /odom;
2. **Node2_service**: A service node that, when called, returnsthe coordinates of the last target sent by the user;

## Nodes
### Node1 description
* File name: Node1.cpp
* Key Features:
Defined some function :
  1. ```void targetCallback(const assignment_2_2024::PlanningGoal& goal)``` this function is called when the user set a new target so it will update the 2 last_target *x* and *y* and store them on the parameter server.
  2. ```void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher pub)``` so this functions is fundamental cause extract the robot's position amd velocities from a message from the /odom topic and publishes it as a custom message.  
  3. ```bool getLastTarget(assignment2_rt::ReturnLastTarget::Request &req, assignment2_rt::ReturnLastTarget::Response &res)``` return last target set by users (it is a check if everything work fine).
  4. ```void stateCallback(const actionlib::SimpleClientGoalState &state, const assignment_2_2024::PlanningResultConstPtr &result)``` , ```void goalCallback()``` and ```void feedbackCallback(const assignment_2_2024::PlanningFeedbackConstPtr &feedback)``` in order : handles the state of the reachment of the goal(if succesfull or not), triggered if the goal is activated and t last one is triggered periodically to report feedback on the robot's progress toward the target.  
Then inside the *main* :  
The node is initialized with ros::init and also an action client ```SimpleActionClient``` is created to communicate with the action server. A subscriber is created for the /odom topic to receive robot position and velocity, as said before.  
A service is advertised with ```advertiseService```  and allow other nodes to request the last target coordinates.
The user must set a target (choosing [1]) or cancel it [2]. If a target is set, the action goal is sent, and the targetCallback is used to update the target coordinates.

### Node2_service description
This node defines a ROS service node that provides the last set target coordinates.  
* File name: Node2_service.cpp
* Key Features:  
Defined a ```bool getLastTarget(assignment2_rt::ReturnLastTarget::Request &req, assignment2_rt::ReturnLastTarget::Response &res)```this functions handles service requests, it checks the parameter server for the values of *last_x_target* and *last_y_target* and assigns them to the service response *res.x* and *res.y*.  
If successful, it returns true; otherwise false.  
Inside the main the most relevant features is that advertise the service named *get_last_target* that will use the *getLastTarget* function to serve requests.


## How to run it
1. Launch the gazebo simulation with the command ```roslaunch assignment_2_2024 assignment1.launch``` (obviusly is requested to have the package ```assignment_2_2024```action server with the robot simulation environment installed in the workspace with the other package).
2. Run Node1 by using ```rosrun assignment2_rt Node1``` it will ask to the user to choose [1] and set a target goal *(x,y)* to be reached by the robot, and [2] to cancel the target.
If is set a target, while the robot is on his way to reach the target goal, typing 2 in the terminal and sending this input will automatically cancel the target and stop the robot.  Then, pressing 1 will allow you to select a new target.
3. Run Node2_service by using ```rosrun assignment2_rt Node2_service```, and in the terminal it s possible to visualize *[INFO] Service 'get_last_target' is ready.*.  At this point in another tab of the terminal is possible to call ```rosservice call /get_last_target "{}"``` and visualizing by monitor the last target set.
