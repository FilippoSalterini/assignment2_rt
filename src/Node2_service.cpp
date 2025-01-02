/*-(b) A service node that, when called, returns the coordinates of the last target sent by the user*/
#include "ros/ros.h"
#include "assignment2_rt/ReturnLastTarget.h"
 
 /*
//Service callback for returtnig last coordinates from the target set bY NODE1
bool getLastTarget(assignment2_rt::ReturnLastTarget::Request &req, assignment2_rt::ReturnLastTarget::Response &res) {
    res.x = last_x_target;
    res.y = last_y_target;

    ROS_INFO("Returning last target coordinates: x = %.2f, y = %.2f", res.x, res.y); //monitor check to see if returning last target
    return true;
}
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "Node2_service");
    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<assignment2_rt::ReturnLastTarget>("get_last_target");

    assignment2_rt::ReturnLastTarget srv;
    ros::Rate loop_rate(1); // Adjust the frequency as needed

    while (ros::ok()) {
        if (client.call(srv)) {
           ROS_INFO("Last target set - x: %.2f, y: %.2f", srv.response.x, srv.response.y);
    }   else {
           ROS_ERROR("Failed to call service get_last_target");
    }
    ros::spinOnce();
    loop_rate.sleep();
}

    return 0;
}
