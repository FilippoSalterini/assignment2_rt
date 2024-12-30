/* A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the 
feedback/status of the action server to know when the target has been reached. The node also publishes the
robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
topic/odom */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>       //needed for creating the action client for communicating with server
#include <nav_msgs/Odometry.h>                    //it s a standard ROS message type to get pos and vel of the robot
#include "assignment_2_2024/PlanningAction.h" 
#include "assignment2_rt/RobotPosition.h"   //my custom message to publish robot position and velocity


using namespace std;

//as request i exctract from odom topic robot (vel and pos) and publish it to my custom msg
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher pub) {
    assignment2_rt::RobotPosition pos;
    
    pos.x = msg->pose.pose.position.x;
    pos.y = msg->pose.pose.position.y;
    pos.vel_x = msg->twist.twist.linear.x;
    pos.vel_z = msg->twist.twist.angular.z;
    
    pub.publish(pos);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_action_client_node"); //initializes ros node as action client node
    ros::NodeHandle nh;


    actionlib::ActionClient<assignment_2_2024::PlanningAction> ac("/reaching_goal", true); //create an action client for the server target action


    ROS_INFO("Waiting for action server to start..."); //waiting act serv to start
    ac.waitForServer();
    
    ROS_INFO("Action server started.");
    
    ros::Publisher position_pub = nh.advertise<assignment2_rt::RobotPosition>("/robot_position", 10); //pub robot pos information to robot_pos topic.

    
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, 
        boost::bind(odomCallback, _1, position_pub)); //it subscribes to odom topic for receiving the robot's odometry data.
        //CHECK BOOST::BIND
        //boot::bind used for pass the position_pub publisher to the odomCallback


    ros::Rate loop_rate(10);
    
     while (ros::ok()) {
     //here implemented the target's choice
        int choice;
        ROS_INFO("Enter (1) to set a target or (2) to cancel a target: ");
        cin >> choice;

        if (choice == 1) {
            float target_x, target_y;
            ROS_INFO("Enter a target coordinates (x, y): ");
            cin >> target_x; 
            cin >> target_y;

            assignment_2_2024::TargetGoal goal; //here i fill an object TargetGoal goal and i fill it with the target values
            goal.target_pose.x = target_x;
            goal.target_pose.y = target_y;
            
            ac.sendGoal(goal,
                        [](const actionlib::SimpleClientGoalState& state,
                           const assignment_2_2024::TargetResultConstPtr& result) {
                            ROS_INFO("Target reached. Final state: %s", state.toString().c_str());
                        },
                        actionlib::SimpleActionClient<assignment_2_2024::PlanningAction>::SimpleActiveCallback(),
                        [](const assignment_2_2024::TargetFeedbackConstPtr& feedback) {
                            ROS_INFO("Current position: (%.2f, %.2f)", feedback->current_pose.x, feedback->current_pose.y);
                        });
        } else if (choice == 2) {
            ac.cancelGoal();
            ROS_INFO("Goal canceled.");
        } else {
            ROS_WARN("Invalid choice.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
 
