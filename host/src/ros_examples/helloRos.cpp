// This is a ROS version of “hello, world”, and was lifted from
// A Gentle Introduction to ROS by Jason O’Kane.

// Define the standard ROS classes.
#include <ros/ros.h>

int main(int argc, char **argv) {
    // Initialize the ROS system.
    ros::init(argc, argv, "hello_ros");

    // Create the ROS node.
    ros::NodeHandle nh;

    // Write the output log message.
    ROS_INFO_STREAM("Hello, ROS World!");
}