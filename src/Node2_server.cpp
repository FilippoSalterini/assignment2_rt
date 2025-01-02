/*-(b) A service node that, when called, returns the coordinates of the last target sent by the user*/

#include "ros/ros.h"
#include "assignment2_rt/ReturnsLastTarget.h"
#include "assignment2_rt/RobotPosition.h"

float last_x_target = 0.0; //here i define some global variable to store time by time last target
float last_y_target = 0.0;

//i take from RobotPosition msg the coordinate from the target set and put them in last target value 
void robotPositionCallback(const assignment2_rt::RobotPosition::ConstPtr& msg) {
    last_x_target = msg->x;
    last_y_target = msg->y;
}
// Service callback for returtnig last coordinates from the target set bY NODE1
bool getLastTarget(assignment2_rt::GetLastTarget::Request &req,
                   assignment2_rt::GetLastTarget::Response &res) {
    res.x = last_target_x;
    res.y = last_target_y;

    ROS_INFO("Returning last target coordinates: x = %.2f, y = %.2f", res.x, res.y);
    return true;
}
