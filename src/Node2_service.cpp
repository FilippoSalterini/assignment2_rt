/**
* \file Node2_service.cpp
* \author Filippo Salterini
* \version 0.1
* \date 31/03/2025
*
* \brief This ROS node acts as an action client to communicate with an action server and provides the last set target coordinates.
**/


#include "ros/ros.h"
#include "assignment2_rt/ReturnLastTarget.h"

/**
 * \brief This functions handles service requests, it checks the parameter server for the values of last_x_target and last_y_target
 *   and assigns them to the service response res.x and res.y.
 * \details Retrieves the last target coordinates (x, y) from the ROS parameter server.
 * \param req The service request.
 * \param res The service response containing the last target coordinates.
 * \return True if the last target was successfully retrieved, false otherwise.
 **/
bool getLastTarget(assignment2_rt::ReturnLastTarget::Request &req, 
                   assignment2_rt::ReturnLastTarget::Response &res) {
    if (ros::param::get("last_x_target", res.x) && ros::param::get("last_y_target", res.y)) {
        ROS_INFO("Returning last target coordinates: x = %.2f, y = %.2f", res.x, res.y);
        return true;
    } else {
        ROS_ERROR("Failed to retrieve last target from parameter server");
        return false;
    }
}

int main(int argc, char** argv) {
    /**
     * \brief Initialize the ROS node.
     * \param argc Number of command-line arguments.
     * \param argv Array of command-line arguments.
     **/
    ros::init(argc, argv, "Node2_service");

    /**
     * \brief Create a node handle.
     **/
    ros::NodeHandle nh;

    /**
     * \brief Advertise the "get_last_target" service.
     * \details This service allows clients to request the last known target coordinates.
     **/
    ros::ServiceServer service = nh.advertiseService("get_last_target", getLastTarget);

    ROS_INFO("Service 'get_last_target' is ready.");

    /**
     * \brief Keep the node running and process service requests.
     **/
    ros::spin();

    return 0;
}

