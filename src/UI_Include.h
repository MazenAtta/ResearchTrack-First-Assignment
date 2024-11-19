#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

std_msgs::String turtle_name_msg;
geometry_msgs::Twist vel_msg;
std::string turtle_name;
float linear, angular;

// Callback for turtle1 pose
void turtle1_Callback(const turtlesim::Pose::ConstPtr& msg) {
    ROS_INFO("Turtle1 Pose: x=%f, y=%f, theta=%f", msg->x, msg->y, msg->theta);
}

// Callback for turtle2 pose
void turtle2_Callback(const turtlesim::Pose::ConstPtr& msg) {
    ROS_INFO("Turtle2 Pose: x=%f, y=%f, theta=%f", msg->x, msg->y, msg->theta);
}

void turtleSpawner()
{
}

void takeUserInput()
{
}

void moveTurtle() 
{
}