#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <string>

ros::Publisher distance_pub;
ros::Publisher turtle1_pub;
ros::Publisher turtle2_pub;
turtlesim::Pose turtle1_pose, turtle2_pose;
geometry_msgs::Twist stop_msg;
geometry_msgs::Twist move_msg;

int turtles_stop_flag = 0;
int turtle1_boundary_stop_flag = 0;
int turtle2_boundary_stop_flag = 0;

void turtle1Callback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = *msg;
}

void turtle2Callback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pose = *msg;

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_distance");
    ros::NodeHandle n;

    distance_pub = n.advertise<std_msgs::Float32>("/turtle_distance", 10);
    turtle1_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    turtle2_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    ros::Subscriber turtle1_sub = n.subscribe("/turtle1/pose", 10, turtle1Callback);
    ros::Subscriber turtle2_sub = n.subscribe("/turtle2/pose", 10, turtle2Callback);
    
    // Reset state flags at the start
    turtles_stop_flag = 0;
    turtle1_boundary_stop_flag = 0;
    turtle2_boundary_stop_flag = 0;
    
    ros::Rate loop_rate(10);  // 10 Hz

    while (ros::ok()) {
    
    // Calculate distance between turtles
    float distance = std::sqrt(std::pow(turtle1_pose.x - turtle2_pose.x, 2) +
                               std::pow(turtle1_pose.y - turtle2_pose.y, 2));

    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    distance_pub.publish(distance_msg);

    // Stop turtles if they are too close to each other or if turtle1 or turtle2 is too close to the boundary
    if (distance < 1.0) {
        turtles_stop_flag = 1;
        
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        turtle1_pub.publish(stop_msg);
        turtle2_pub.publish(stop_msg);
        ROS_WARN("Stopping turtles: too close.");
    }

    else if (turtle1_pose.x < 1.0 || turtle1_pose.x > 10.0 || turtle1_pose.y < 1.0 || turtle1_pose.y > 10.0) {
        turtle1_boundary_stop_flag = 1;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        turtle1_pub.publish(stop_msg);
        ROS_WARN("Stopping turtle1: too close to boundary.");}
    
    else if (turtle2_pose.x < 1.0 || turtle2_pose.x > 10.0 || turtle2_pose.y < 1.0 || turtle2_pose.y > 10.0) {
        turtle2_boundary_stop_flag = 1;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        turtle2_pub.publish(stop_msg);
        ROS_WARN("Stopping turtle2: too close to boundary.");
    }
    else 
    {
        // do nothing
    }
    

    //Move turtles in opposite directions to avoid collision
    if(turtles_stop_flag == 1)
    {
        turtles_stop_flag = 0;
        
        move_msg.angular.z = M_PI_2;
        ros::Duration(1.0).sleep();
        move_msg.linear.x = 1.0;
        ros::Duration(1.0).sleep();
        turtle1_pub.publish(move_msg);
        turtle2_pub.publish(move_msg);
        ROS_WARN("Moving turtles: to avoid collision.");

    }
    else if(turtle1_boundary_stop_flag == 1)
    {
        turtle1_boundary_stop_flag = 0;
        move_msg.angular.z = M_PI_2;
        ros::Duration(1.0).sleep();
        move_msg.linear.x = 1.0;
        ros::Duration(1.0).sleep();
        turtle1_pub.publish(move_msg);
        ROS_WARN("Moving turtle1: to avoid boundary.");
    }
    else if(turtle2_boundary_stop_flag == 1)
    {
        turtle2_boundary_stop_flag = 0;
        ros::Duration(1.0).sleep();
        move_msg.angular.z = M_PI_2;
        move_msg.linear.x = 1.0;
        ros::Duration(1.0).sleep();
        turtle2_pub.publish(move_msg);
        ROS_WARN("Moving turtle2: to avoid boundary.");
    }



    ros::spinOnce();

    }
    return 0;
}
