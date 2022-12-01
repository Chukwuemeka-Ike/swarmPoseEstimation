#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <stdlib.h> // For rand().

int main (int argc, char **argv) {
    // Initialize the ROS system and create the node.
    ros::init(argc, argv, "average_publisher");
    ros::NodeHandle nh;

    // Create the publisher object.
    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/average", 1000);

    // Seed the random number generator.
    srand(time(0));

    // Loop at 0.5 Hz until the node is shut down.
    ros::Rate rate(0.5);

    while(ros::ok()) {
        // Create the message and fill it in with 3 random numbers
        // between 0 and 100.
        geometry_msgs::Vector3 vec;
        vec.x = int(double(rand())/double(RAND_MAX)*100);
        vec.y = int(double(rand())/double(RAND_MAX)*100);
        vec.z = int(double(rand())/double(RAND_MAX)*100);

        // Publish the message.
        pub.publish(vec);

        // Send a message to rosout with the details.
        ROS_INFO_STREAM("Sending random vector with:\n"
            << "\t\tx = " << vec.x << " "
            << "y = " << vec.y << " "
            << "z = " << vec.z
        );

        // Wait until the next iteration.
        rate.sleep();
    }

}