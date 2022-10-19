/**
 * Takes a photo each second, calculates the robot's position, publishes it.
 * 
 * This program uses OpenCV and Aruco tags to continuously estimate the
 * position of the Khepera in the map. Once the estimation is done, it
 * publishes this information on the /khepera_pose topic.
 * 
 * Usage:
 *  ./posePublisher <server_ip>:<port_num> <khepera_ip>
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

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
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
  cout << argc << endl;
  if (argc < 3){
    cerr << "ERROR: Only " << argc
          << " arguments specified. Cannot create ROS node\n";
    return -1;
  }

  // Get server and khepera IP addresses from the args.
  char* my_ip = argv[2];
  char* server_ip = argv[1];
  cout << "Connecting Khepera with address " <<  my_ip
        << " to Server at address " << server_ip << endl;

  // Set up the ROS node and publisher topic.
  nh.initNode(server_ip);
  nh.advertise(posePublisher);
  cout << "Initialized ROS node and created topic.\n";

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
  my_pose.child_frame_id = my_ip;

  while (1) {
    captureInputImage(capture_command, capture_file, inputImage);

    estimateKheperaPose(
      inputImage, outputImage, cameraMatrix, distCoeffs, rotMatrix, 
      intermediate, padding, transCamToTag, transCamToWorld, rvecs, tvecs,
      markerIds, dictionary, parameters,
      markerCorners, rejectedCandidates, tagLocations, my_pose,
      output_image_file
    );

    // Publish the updated pose.
    posePublisher.publish(&my_pose);
    
    // Spin node and sleep for 1 second.
    nh.spinOnce();
    sleep(1);
  }
}