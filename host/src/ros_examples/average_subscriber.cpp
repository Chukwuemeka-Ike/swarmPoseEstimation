#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <iomanip> // for std::setprecision and std::fixed.

// The callback for each message received.
void inputMessageReceived(const geometry_msgs::Vector3& msg) {
    ROS_INFO_STREAM(std::setprecision(4) << std::fixed
        << "The average of \n\t\t" << msg.x << ", " << msg.y << ", and " << msg.z
        << " is: " << double((msg.x+msg.y+msg.z)/3) 
    );
}

int main(int argc, char **argv) {
    //Initialize the ROS system and create the node.
    ros::init(argc, argv, "average_subscriber");
    ros::NodeHandle nh;

    // Create the subscriber object.
    ros::Subscriber sub = nh.subscribe("/average", 1000, &inputMessageReceived);

    // Give ROS control.
    ros::spin();
}