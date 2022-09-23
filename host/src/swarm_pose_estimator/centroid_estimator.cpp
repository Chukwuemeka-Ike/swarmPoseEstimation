/*  Node for calculating the swarm centroid as Kheperas publish their poses.

    This spawns a node that subscribes to the khepera_pose topic and
    calculates the swarm's centroid once it receives poses from every
    robot in the swarm.
*/
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <stdlib.h>
#include <iostream>
#include <unordered_set>

using namespace std;

/*
    Class that takes care of subscribing to the khepera_pose topic, calculating
    swarm centroid, and publishing the result on the swarm_centroid topic.
*/
class CentroidEstimator
{

    public: CentroidEstimator(int swarm_size)
    {
        // Create subscriber and publisher.
        sub_ = nh_.subscribe("khepera_pose", 1000,
                &CentroidEstimator::poseReceived, this);
        pub_ = nh_.advertise<geometry_msgs::Point>("swarm_centroid", 1000);

        // Set swarm_size and initial centroid values.
        swarm_size_ = swarm_size;
        x_sum_ = 0.0;
        y_sum_ = 0.0;
        z_sum_ = 0.0;
    }

    // Callback function for 
    void poseReceived(const geometry_msgs::TransformStamped& kheperaPose) {
        ROS_DEBUG_ONCE("kheperaPose received.");

        // If received IP is not in set, add it and the bot's translation info.
        string ip = kheperaPose.child_frame_id;
        if (ip_addresses_.find(ip) == ip_addresses_.end()) {
            // Insert the ip address.
            ip_addresses_.insert(ip);

            // Add the bot's translation info.
            x_sum_ = x_sum_ + kheperaPose.transform.translation.x;
            y_sum_ = y_sum_ + kheperaPose.transform.translation.y;
            z_sum_ = z_sum_ + kheperaPose.transform.translation.z;
        }

        // If we have info from the whole swarm, get centroid and publish.
        if (ip_addresses_.size() == swarm_size_) {
            swarm_centroid_.x = x_sum_ / swarm_size_;
            swarm_centroid_.y = y_sum_ / swarm_size_;
            swarm_centroid_.z = z_sum_ / swarm_size_;

            pub_.publish(swarm_centroid_);

            // Clear IP set and sums.
            ip_addresses_.clear();
            x_sum_ = 0.0;
            y_sum_ = 0.0;
            z_sum_ = 0.0;
        }
    }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        geometry_msgs::Point swarm_centroid_;
        unordered_set<string> ip_addresses_;
        int swarm_size_;
        double x_sum_, y_sum_, z_sum_;
};

int main(int argc, char *argv[]) {
    int swarm_size = atoi(argv[1]);
    cout << "Running centroid estimation for swarm of size "
        << swarm_size << "." << endl;

    // Initialize ROS node.
    ros::init(argc, argv, "centroid_estimator");

    // Create centroid estimator object.
    CentroidEstimator centroidEstimator(swarm_size);
    ros::spin();

    return 0;
}