/* A node that implements an action client, allowing the userto set a target (x, y) or to cancel it. Try to use the 
feedback/status of the action server to know when the target has been reached. The node also publishesthe
robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
topic /odom */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2024/TargetAction.h>
#include <nav_msgs/Odometry.h>
#include <custom_msgs/RobotState.h> //my custom message to publish robot position and velocity


