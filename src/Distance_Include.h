#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

ros::Publisher distance_pub;
ros::Publisher turtle1_pub;
ros::Publisher turtle2_pub;
turtlesim::Pose turtle1_pose, turtle2_pose;
std_msgs::Float32 distance_msg;
std::string last_moving_turtle = "";



geometry_msgs::Twist stop_msg;
geometry_msgs::Twist move_msg;

int turtles_stop_flag = 0;
int turtle1_boundary_stop_flag = 0;
int turtle2_boundary_stop_flag = 0;
float distance = 0.0;

void turtle1Callback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = *msg;
}

void turtle2Callback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pose = *msg;
}

void lastMovingTurtleCallback(const std_msgs::String::ConstPtr& msg) {
    last_moving_turtle = msg->data;
}

float calculateDistance(const turtlesim::Pose& pose1, const turtlesim::Pose& pose2) {
    return std::sqrt(std::pow(pose1.x - pose2.x, 2) + std::pow(pose1.y - pose2.y, 2));
}

// Function to Stop a Turtle
void stopTurtle(ros::Publisher& pub, const std::string& turtle_name) {
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    pub.publish(stop_msg);
    ROS_WARN("Stopping %s", turtle_name.c_str());
}

// Function to Move a Turtle to a Safe Space
void moveTurtleToSafeSpace(ros::Publisher& pub, const std::string& turtle_name) {
    move_msg.angular.z = M_PI;
    move_msg.linear.x = 0.0;
    pub.publish(move_msg);
    ros::Duration(1.0).sleep();

    move_msg.angular.z = 0.0;
    move_msg.linear.x = 1.0;
    pub.publish(move_msg);
    ros::Duration(1.0).sleep();

    ROS_WARN("Moving %s to a safe space.", turtle_name.c_str());
}

// Function to Check Boundary Conditions
bool isTurtleSafe(const turtlesim::Pose& pose, const std::string& turtle_name, float min_distance = 1.0) {
    bool nearBoundary(pose.x < 1.0 || pose.x > 10.0 || pose.y < 1.0 || pose.y > 10.0);
    
    // Check if the turtle is near the boundary
    if (nearBoundary)
    {
        ROS_WARN("%s is near the boundary. Moving to a safe space.", turtle_name.c_str());
        return true;
    }

    // Check if the turtle is too close to the other turtle
    if (distance < min_distance) {
        ROS_WARN("%s is too close to the other turtle. Distance: %f", turtle_name.c_str(), distance);
        return true;
    }
    return false;
}
