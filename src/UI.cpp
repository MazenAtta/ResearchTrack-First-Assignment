#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h"

// Callback for turtle1 pose
void turtle1_Callback(const turtlesim::Pose::ConstPtr& msg) {
    ROS_INFO("Turtle1 Pose: x=%f, y=%f, theta=%f", msg->x, msg->y, msg->theta);
}

// Callback for turtle2 pose
void turtle2_Callback(const turtlesim::Pose::ConstPtr& msg) {
    ROS_INFO("Turtle2 Pose: x=%f, y=%f, theta=%f", msg->x, msg->y, msg->theta);
}

int main(int argc, char **argv) {
    // Initialize the ROS system and create a node handle
    ros::init(argc, argv, "UI");
    ros::NodeHandle n;
    geometry_msgs::Twist vel_msg;

    // Spawn turtle2
    ros::ServiceClient spawn_client = n.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 2.0;
    spawn_srv.request.y = 1.0;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle2";
    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Spawned turtle2 at (2.0, 1.0, 0.0)");
    } else {
        ROS_ERROR("Failed to spawn turtle2");
    }

    // Define publishers and subscribers for turtle1 and turtle2
    ros::Subscriber turtle1_sub = n.subscribe("/turtle1/pose", 10, turtle1_Callback);
    ros::Publisher turtle1_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::Subscriber turtle2_sub = n.subscribe("/turtle2/pose", 10, turtle2_Callback);
    ros::Publisher turtle2_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    std::string turtle_name;
    float linear, angular;

    while (ros::ok()) {
        std::cout << "Select turtle (turtle1/turtle2): ";
        std::cin >> turtle_name;

        if (turtle_name != "turtle1" && turtle_name != "turtle2") {
            std::cout << "Invalid turtle name. Try again.\n";
            continue;
        }

        std::cout << "Enter linear velocity: ";
        std::cin >> linear;
        std::cout << "Enter angular velocity: ";
        std::cin >> angular;

        vel_msg.linear.x = linear;
        vel_msg.angular.z = angular;

        if (turtle_name == "turtle1") {
            turtle1_pub.publish(vel_msg);
        } else if (turtle_name == "turtle2") {
            turtle2_pub.publish(vel_msg);
        } else {
            std::cout << "Invalid turtle name. Try again.\n";
            continue;
        }

        // Send the command for 1 second
        ros::Duration(1.0).sleep();

        // Stop the turtle
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        if (turtle_name == "turtle1") {
            turtle1_pub.publish(vel_msg);
        } else if (turtle_name == "turtle2") {
            turtle2_pub.publish(vel_msg);
        }
    }

    return 0;
}
