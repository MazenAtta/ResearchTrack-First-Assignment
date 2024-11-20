#include "Assignment1/Distance_Include.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_distance");
    ros::NodeHandle n;


    distance_pub = n.advertise<std_msgs::Float32>("/turtle_distance", 10);
    turtle1_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    turtle2_pub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);


    ros::Subscriber turtle1_sub = n.subscribe("/turtle1/pose", 10, turtle1Callback);
    ros::Subscriber turtle2_sub = n.subscribe("/turtle2/pose", 10, turtle2Callback);
    ros::Subscriber last_moving_turtle_sub = n.subscribe("/last_moving_turtle", 10, lastMovingTurtleCallback);


    // Wait for initial pose updates
    ROS_INFO("Waiting for turtle1 and turtle2 pose updates...");
    
    ros::Rate rate(10); // 10 Hz loop
    bool poses_received = false;
    
    while (ros::ok() && !poses_received) {
        ros::spinOnce(); // Process callbacks

        // Check if both poses are non-zero (indicating they've been updated)
        if (turtle1_pose.x != 0.0 || turtle1_pose.y != 0.0 || turtle2_pose.x != 0.0 || turtle2_pose.y != 0.0) {
            poses_received = true;
        }
        rate.sleep();
    }
    
    ROS_INFO("Pose updates received. Starting main loop.");

    while (ros::ok()) 
    {
        ros::spinOnce();
        distance = calculateDistance(turtle1_pose, turtle2_pose);
        distance_msg.data = distance;
        distance_pub.publish(distance_msg);
        
        // Check if the turtles are in safe spaces
        if(isTurtleSafe(turtle1_pose, "turtle1") && last_moving_turtle == "turtle1") 
        {
            stopTurtle(turtle1_pub, "turtle1");
            moveTurtleToSafeSpace(turtle1_pub, "turtle1");
        }
        else if(isTurtleSafe(turtle2_pose, "turtle2") && last_moving_turtle == "turtle2"){
            stopTurtle(turtle2_pub, "turtle2");
            moveTurtleToSafeSpace(turtle2_pub, "turtle2");
        }
    }
    return 0;
}
