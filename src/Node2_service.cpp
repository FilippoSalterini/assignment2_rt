/*-(b) A service node that, when called, returns the coordinates of the last target sent by the user*/
#include "ros/ros.h"
#include "assignment2_rt/ReturnLastTarget.h"
#include "assignment2_rt/RobotPosition.h"

float last_x_target = 0.0; //here i define some global variable to store time by time last target
float last_y_target = 0.0;

//i take from RobotPosition msg the coordinate from the target set and put them in last target value 
void robotPositionCallback(const assignment2_rt::RobotPosition::ConstPtr& msg) {
    last_x_target = msg->x;
    last_y_target = msg->y;
    ROS_INFO("Updated target coordinates: x = %.2f, y = %.2f", last_x_target, last_y_target); // Add this line

}

// Service callback for returtnig last coordinates from the target set bY NODE1
bool getLastTarget(assignment2_rt::ReturnLastTarget::Request &req, assignment2_rt::ReturnLastTarget::Response &res) {
    res.x = last_x_target;
    res.y = last_y_target;

    ROS_INFO("Returning last target coordinates: x = %.2f, y = %.2f", res.x, res.y); //monitor check to see if returning last target
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Node2_service");
    ros::NodeHandle nh;

    // sub to robot_position topic
    ros::Subscriber sub = nh.subscribe("robot_position", 10, robotPositionCallback);

    ros::ServiceServer service = nh.advertiseService("get_last_target", getLastTarget);
    ROS_INFO("Service node ready to provide last target coordinates.");

    ros::spin();
    return 0;
}
