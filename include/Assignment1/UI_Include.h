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

void turtleSpawner(ros::NodeHandle& n, std::string turtle_name, float x = 0, float y = 0, float theta = 0)
{
    ros::ServiceClient spawn_client = n.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = x;
    spawn_srv.request.y = y;
    spawn_srv.request.theta = theta;
    spawn_srv.request.name = turtle_name;
    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Spawned %s at (2.0, 1.0, 0.0)", turtle_name.c_str());
    } 
    else 
    {
     ROS_ERROR("Failed to spawn %s", turtle_name.c_str());
    }
}
std::string takeUserInput()
{

    std::cout << "Select turtle (turtle1/turtle2): ";
    std::cin >> turtle_name;
    
    
    if (turtle_name != "turtle1" && turtle_name != "turtle2") 
    {
        std::cout << "Invalid turtle name. Try again.\n";
        return "";
    }

    std::cout << "Enter linear velocity: ";
    std::cin >> linear;
    std::cout << "Enter angular velocity: ";
    std::cin >> angular;

    vel_msg.linear.x = linear;
    vel_msg.angular.z = angular;

    return turtle_name;
}
