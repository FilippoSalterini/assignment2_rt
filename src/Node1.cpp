/**
* \file Node1.cpp
* \author Filippo Salterini
* \version 0.1
* \date 31/03/2025
*
* \brief A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the feedback/status of the action server to know
*   when the target has been reached. The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying * on the values
*   published on the topic /odom.
*
* \details
*  Subscribes to:
*    /odom (nav_msgs::Odometry): Receives the robot's position and velocity.
*  Publishes to:
*   /robot_position (assignment2_rt::RobotPosition): Publishes the robot's position and velocity.
*  Provides Services:
*   /get_last_target (assignment2_rt::ReturnLastTarget): Returns the last target set.
*  Uses Action Clients:
*   /reaching_goal (assignment_2_2024::PlanningAction): Sends a goal to the robot.
**/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include "assignment_2_2024/PlanningAction.h"
#include "assignment2_rt/RobotPosition.h"
#include "assignment2_rt/ReturnLastTarget.h"

using namespace std;

float last_x_target = 0.0;
float last_y_target = 0.0;

/**
* \brief This function is called when the user set a new target so it will update the 2 last_target x and y and store them on the parameter server.
* \param goal The goal received, containing the target coordinates.
*/
void targetCallback(const assignment_2_2024::PlanningGoal& goal) {
    last_x_target = goal.target_pose.pose.position.x;
    last_y_target = goal.target_pose.pose.position.y;
    ros::param::set("/last_x_target", last_x_target);
    ros::param::set("/last_y_target", last_y_target);
    ROS_INFO("Updated last target to x = %.2f, y = %.2f", last_x_target, last_y_target);
}

/**
* \brief This functions extract the robot's position and velocities from a message from Odometry data topic and publishes it as a custom message.
* \param msg The received odometry message.
* \param pub The publisher to publish robot position and velocity.
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher pub) {
    assignment2_rt::RobotPosition robot_position;
    robot_position.x = msg->pose.pose.position.x;
    robot_position.y = msg->pose.pose.position.y;
    robot_position.vel_x = msg->twist.twist.linear.x;
    robot_position.vel_z = msg->twist.twist.angular.z;
    pub.publish(robot_position);
}

/**
* \brief Service callback that return last target set by users (it is a check if everything work fine).
* \param req The service request.
* \param res The service response containing the last target coordinates.
* \return Always true.
*/
bool getLastTarget(assignment2_rt::ReturnLastTarget::Request &req, assignment2_rt::ReturnLastTarget::Response &res) {
    res.x = last_x_target;
    res.y = last_y_target;
    ROS_INFO("Returning last target coordinates: x = %.2f, y = %.2f", res.x, res.y);
    return true;
}

/**
* \brief Callback for action result state.
* \param state The final state of the action.
* \param result The result returned by the action.
*/
void stateCallback(const actionlib::SimpleClientGoalState &state, const assignment_2_2024::PlanningResultConstPtr &result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Target reached successfully!");
    } else {
        ROS_WARN("Target failed with state: %s", state.toString().c_str());
    }
}

/**
* \brief Callback triggered when the goal becomes active.
*/
void goalCallback() {
    ROS_INFO("Goal is now active.");
}

/**
* \brief Callback for action feedback.
* \param feedback The feedback message containing the current position.
*/
void feedbackCallback(const assignment_2_2024::PlanningFeedbackConstPtr &feedback) {
    ROS_INFO("Feedback received: Current position -> x: %.2f, y: %.2f", feedback->actual_pose.position.x, feedback->actual_pose.position.y);
}

/**
* \brief Main function to initialize the ROS node and handle user input.
**/

int main(int argc, char** argv) {
    ros::init(argc, argv, "Node1"); ///< Initialize the ROS node with the name "Node1".
    ros::NodeHandle nh;

    /**
     * \brief Create an action client to communicate with the "reaching_goal" action server.
     **/
    actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> ac("reaching_goal", true);

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    /**
     * \brief Create a publisher to publish the robot's position on the "robot_position" topic.
     **/
    ros::Publisher position_pub = nh.advertise<assignment2_rt::RobotPosition>("robot_position", 10);

    /**
     * \brief Subscribe to the "/odom" topic to receive odometry data.
     * \param msg The odometry message.
     **/
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10,
        [&position_pub](const nav_msgs::Odometry::ConstPtr &msg) {
            odomCallback(msg, position_pub);
        });

    /**
     * \brief Advertise a service to return the last target set by the user.
     **/
    ros::ServiceServer service = nh.advertiseService("get_last_target", getLastTarget);

    ros::Rate loop_rate(10); ///< Set the loop rate to 10 Hz.

    /**
     * \brief Main loop: keeps running as long as ROS is active.
     **/
    while (ros::ok()) {
        int choice;
        ROS_INFO("Enter (1) to set a target or (2) to cancel a target: ");
        cin >> choice;

        if (choice == 1) {
            /**
             * \brief User selects to set a new target.
             * \details The user is prompted to enter the target coordinates (x, y).
             *          A goal message is created and sent to the action server.
             **/
            float target_x, target_y;
            ROS_INFO("Enter target coordinates (x, y): ");
            cin >> target_x >> target_y; ///< Read target coordinates from user input.

            assignment_2_2024::PlanningGoal goal;
            goal.target_pose.pose.position.x = target_x;
            goal.target_pose.pose.position.y = target_y;
            goal.target_pose.pose.orientation.w = 1.0;

            /**
             * \brief Send the goal to the action server.
             * \param goal The goal message containing target coordinates.
             **/
            ac.sendGoal(goal, &stateCallback, &goalCallback, &feedbackCallback);

            /**
             * \brief Update the last known target.
             * \param goal The goal message containing target coordinates.
             **/
            targetCallback(goal);

        } else if (choice == 2) {
            /**
             * \brief User selects to cancel the current goal.
             * \details The goal is canceled by calling the cancelGoal() function.
             **/
            ROS_INFO("User selected to cancel the goal.");
            ac.cancelGoal();
            ROS_INFO("Goal canceled.");
        } else {
            /**
             * \brief Handle invalid user input.
             **/
            ROS_WARN("Invalid choice.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

