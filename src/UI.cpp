#include "Assignment1/UI_Include.h"






int main(int argc, char **argv) 
{
    // Initialize the ROS system and create a node handle
    ros::init(argc, argv, "UI");
    ros::NodeHandle n;

    // Spawn turtle2
    turtleSpawner(n, "turtle2", 2.0, 1.0, 0.0);

    // Define publishers and subscribers for turtle1 and turtle2
    ros::Subscriber turtle1_sub = n.subscribe("/turtle1/pose", 10, turtle1_Callback);
    ros::Publisher turtle1_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::Subscriber turtle2_sub = n.subscribe("/turtle2/pose", 10, turtle2_Callback);
    ros::Publisher turtle2_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    ros::Publisher last_moving_turtle_pub = n.advertise<std_msgs::String>("/last_moving_turtle", 10);

    ros::Rate rate(10); // 10 Hz loop

    while (ros::ok()) 
    {
        ros::spinOnce();
        // Take user input for turtle name, linear velocity, and angular velocity
        turtle_name = takeUserInput();
        

        if (turtle_name == "turtle1") 
        {
            turtle1_pub.publish(vel_msg);
        } 
        else if (turtle_name == "turtle2") 
        {
            turtle2_pub.publish(vel_msg);
        } 
        else 
        {
            std::cout << turtle_name;
            continue;
        }

        turtle_name_msg.data = turtle_name;  // Set the name of the last moving turtle
        last_moving_turtle_pub.publish(turtle_name_msg);  // Publish it

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
