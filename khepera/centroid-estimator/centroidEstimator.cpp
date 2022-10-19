/**
 * @brief Estimates the swarm centroid based on its position and the group estimate.
 * 
 * Each Khepera maintains its own estimate of the swarm's centroid, and
 * continuously updates it with its neighbors' estimates. The goal is that
 * after enough iterations, the swarm will know exactly where they are.
 * 
 * Usage:
 *  ./centroidEstimator <server_ip>:<port_num> <khepera_ip>
 */
#include <ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <stdlib.h>
#include <iostream>

#include <swarm_pose_estimation/kheperaPoseEstimation.h>

using namespace std;
using namespace cv;

#define SELF_WEIGHT 0.5
#define TEAM_WEIGHT 0.3

geometry_msgs::TransformStamped my_estimate;
ros::Publisher estimatePublisher("centroid_estimate", &my_estimate);
int numIPs = 0;
string my_ip;

/**
 * @brief Updates the bot's centroid estimate with the received estimate.
 * 
 * @param receivedEstimate the estimate received by this Khepera.
 */
void updateAndPublishEstimate(const geometry_msgs::TransformStamped& receivedEstimate) 
{
  if (receivedEstimate.child_frame_id != my_ip && numIPs == 0) {
    my_estimate.transform.translation.x = float(SELF_WEIGHT*my_estimate.transform.translation.x) + float(TEAM_WEIGHT*receivedEstimate.transform.translation.x);
    my_estimate.transform.translation.y = float(SELF_WEIGHT*my_estimate.transform.translation.y) + float(TEAM_WEIGHT*receivedEstimate.transform.translation.y);
    my_estimate.transform.translation.z = float(SELF_WEIGHT*my_estimate.transform.translation.z) + float(TEAM_WEIGHT*receivedEstimate.transform.translation.z);
    estimatePublisher.publish(&my_estimate);
    numIPs = 1;
  }
}

int main(int argc, char *argv[])
{
  // File names and camera capture command.
  string capture_file = "input_img.jpg";
  string output_image_file = "output_img.jpg";
  string camera_params_file = "cameraData.xml";
  string tag_locations_file = "tagLocations.yaml";
  string capture_command = "v4l2grab -d /dev/video6 -o "
                            + capture_file + " -W 752 -H 480 -q 85 -I -1";

  // Create the ROS node handle, message and publisher.
  ros::NodeHandle nh;
  geometry_msgs::TransformStamped my_pose;
  ros::Publisher posePublisher("khepera_pose", &my_pose);
  ros::Subscriber<geometry_msgs::TransformStamped> sub(
    "centroid_estimate", updateAndPublishEstimate
  );

  // OpenCV and ArUco variables for images, camera parameters, and pose info.
  Mat inputImage, outputImage, cameraMatrix, distCoeffs, rotMatrix;
  Mat transCamToTag, transCamToWorld, intermediate;
  float b[1][4] = {0, 0, 0, 1};
  Mat padding = Mat(1, 4, CV_32F, b);
  map<int, Mat> tagLocations;
  vector<int> markerIds;
  vector<vector<Point2f> > markerCorners, rejectedCandidates;
  vector<Vec3d> rvecs, tvecs;
  Ptr<aruco::DetectorParameters> parameters =
                                      aruco::DetectorParameters::create();
  Ptr<aruco::Dictionary> dictionary =
                    aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

  // Ensure we have the correct set of arguments.
  if (argc < 3){
    cerr << "ERROR: Only " << argc
          << " arguments specified. Cannot create ROS node\n";
    return -1;
  }
  
  // Get server and khepera IP addresses.
  my_ip = argv[2];
  char* server_ip = argv[1];
  cout << "Connecting Khepera with address " <<  my_ip
        << " to Server at address " << server_ip << endl;

  // Set up the ROS node and topics.
  nh.initNode(server_ip);
  nh.advertise(posePublisher);
  nh.advertise(estimatePublisher);
  nh.subscribe(sub);
  cout << "Initialized ROS node and topics.\n";

  // Get the camera parameters and tag locations.
  getTagLocations(tagLocations, tag_locations_file);
  getCameraParams(cameraMatrix, distCoeffs, camera_params_file);

  // Set up camera format.
  cout << "Setting Pipes\n";
  system("media-pipes.sh");
  cout << "Setting Format\n";
  system("media-formats.sh 752 480");

  // Set up my IP.
  my_pose.header.frame_id = "world";
  my_pose.child_frame_id = my_ip.c_str();
  my_estimate.child_frame_id = my_ip.c_str();

  while (1) {
    captureInputImage(capture_command, capture_file, inputImage);

    estimateKheperaPose(
      inputImage, outputImage, cameraMatrix, distCoeffs, rotMatrix, 
      intermediate, padding, transCamToTag, transCamToWorld, rvecs, tvecs,
      markerIds, dictionary, parameters,
      markerCorners, rejectedCandidates, tagLocations, my_pose,
      output_image_file
    );

    my_estimate.transform.translation.x = float(my_estimate.transform.translation.x + my_pose.transform.translation.x) / 2;
    my_estimate.transform.translation.y = float(my_estimate.transform.translation.y + my_pose.transform.translation.y) / 2;
    my_estimate.transform.translation.z = float(my_estimate.transform.translation.z + my_pose.transform.translation.z) / 2;

    // Publish the updated pose.
    posePublisher.publish(&my_pose);
    estimatePublisher.publish(&my_estimate);

    // Spin node and sleep for 2 seconds.
    numIPs = 0;
    nh.spinOnce();
    sleep(2);
  }

}