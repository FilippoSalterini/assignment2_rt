/*-(b) A service node that, when called, returns the coordinates of the last target sent by the user*/
#include "ros/ros.h"
#include "assignment2_rt/ReturnLastTarget.h"
 
/*bool getLastTarget(assignment2_rt::ReturnLastTarget::Request &req, assignment2_rt::ReturnLastTarget::Response &res) {
    // so here i retrieve the last target from the parameter server
    if (ros::param::get("last_x_target", res.x) && ros::param::get("last_y_target", res.y)) {
        ROS_INFO("Returning last target coordinates: x = %.2f, y = %.2f", res.x, res.y);
        return true;
    } else {
        ROS_ERROR("Failed to retrieve last target from parameter server");
        return false;
    }
}
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "Node2_service");
    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<assignment2_rt::ReturnLastTarget>("get_last_target");

    assignment2_rt::ReturnLastTarget srv;
    
    ros::Rate loop_rate(1);

    if (client.call(srv)) {
        ROS_INFO("Last target set - x: %.2f, y: %.2f", srv.response.x, srv.response.y);
    } else {
        ROS_ERROR("Failed to call service get_last_target");
    }

    // here shutdown Node2 after the service call
    ros::shutdown();

    return 0;
}
