/* A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the 
feedback/status of the action server to know when the target has been reached. The node also publishes the
robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the
topic/odom */

#include <ros/ros.h>
#include <actionlib/client/action_client.h>       //needed for creating the action client for communicating with server
#include <assignment_2_2024/TargetActionStatus.h> //it generates automatically an header for the target action
#include <nav_msgs/Odometry.h>                    //it s a standard ROS message type to get pos and vel of the robot
#include <custom_msgs/RobotPosition.h>            //my custom message to publish robot position and velocity

using namespace std;

//as request i exctract from odom topic robot (vel and pos) and publish it to my custom msg
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher pub) {
    custom_msgs::RobotPosition pos;
    
    pos.x = msg->pose.pose.position.x;
    pos.y = msg->pose.pose.position.y;
    pos.vel_x = msg->twist.twist.linear.x;
    pos.vel_z = msg->twist.twist.angular.z;
    
    pub.publish(pos);
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_node"); //initializes ros node as action client node
    ros::NodeHandle nh;


    actionlib::ActionClient<assignment_2_2024::TargetAction> ac("/reaching_goal", true); //create an action client for the server target action


    ROS_INFO("Waiting for action server to start..."); //waiting act serv to start
    ac.waitForServer();
    
    ROS_INFO("Action server started.");
    
    ros::Publisher state_pub = nh.advertise<custom_msgs::Robotp>("/robot_state", 10); //pub robot pos information to robot_pos topic.

    
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, 
        boost::bind(odomCallback, _1, state_pub)); //sub to odom topic


    ros::Rate loop_rate(10);
    
    
    
    
    
    
    
