/*Takes a photo each second, calculates the robot's position, publishes it.

  This program uses OpenCV and Aruco tags to continuously estimate the
  position of the Khepera in the map. Once the estimation is done, it
  publishes this information on the /khepera_pose topic.

  Usage:
  ./posePublisher <server_ip>:<port_num> <khepera_ip>
*/
#include <ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <stdlib.h>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
  // Ensure we have the correct set of arguments.
  cout << argc << endl;
  if (argc < 3){
    cerr << "ERROR: Only " << argc
          << " arguments specified. Cannot create ROS node\n";
    return -1;
  }
  
  // Get server and khepera IP addresses.
  char* my_ip = argv[2];
  char* server_ip = argv[1];
  cout << "Connecting Khepera with address " <<  my_ip
        << " to Server at address " << server_ip << endl;

  // Create the ROS node handle, message and publisher.
  ros::NodeHandle nh;
  geometry_msgs::TransformStamped my_pose;
  ros::Publisher posePublisher("khepera_pose", &my_pose);
  
  // ROS node and topic Setup.
  // nh.initNode();
  nh.initNode(server_ip);
  nh.advertise(posePublisher);
  cout << "Initialized ROS node and created topic.\n";

  // CV matrices to hold the images and necessary camera parameters.
  Mat inputImage, outputImage, cameraMatrix, distCoeffs, rotMatrix;

  // Marker IDs with transformation matrices.
  float a[4][4] = {{1, 0, 0, 0},
                  {0, 1, 0, 0},
                  {0, 0, 1, 0},
                  {0, 0, 0, 1}};
  Mat transTag0ToWorld = Mat(4, 4, CV_32F, a);
  Mat transCamToTag0, transCamToWorld, intermediate;
  float b[1][4] = {0, 0, 0, 1};
  Mat padding = Mat(1, 4, CV_32F, b);

  // Variables for ArUco detection.
  vector<int> markerIds;
  vector<vector<Point2f> > markerCorners, rejectedCandidates;
  Ptr<aruco::DetectorParameters> parameters =
                                      aruco::DetectorParameters::create();
  Ptr<aruco::Dictionary> dictionary =
                    aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
  vector<Vec3d> rvecs, tvecs;

  string capture_file = "input_img.jpg";
  string output_image_file = "output_img.jpg";
  string camera_params_file = "cameraData.xml";
  string capture_command = "v4l2grab -d /dev/video6 -o "
                            + capture_file + " -W 752 -H 480 -q 85 -I -1";

  // Get the camera parameters.
  FileStorage fs(camera_params_file, FileStorage::READ);
  if(!fs.isOpened())
  {
    cerr << "Unable to access camera parameters!" << endl;
    return -1;
  }
  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;
  fs.release();

  // Set up camera format.
  cout << "Setting Pipes\n";
  system("media-pipes.sh");
  cout << "Setting Format\n";
  system("media-formats.sh 752 480");

  // Set up my IP.
  my_pose.header.frame_id = "world";
  my_pose.child_frame_id = my_ip;

  while (1) {
    cout << "Taking Image\n";
    system(capture_command.c_str());

    cout << "Reading image into matrix.\n";
    inputImage = imread(capture_file);

    // Perform detection on the input image
    cout << "Detecting markers.\n";
    aruco::detectMarkers(inputImage, dictionary, markerCorners, 
                                  markerIds, parameters, rejectedCandidates);

    // If at least one marker is detected, we can attempt to
    // estimate the robot's pose.
    if (markerIds.size() > 0) {
      // Copy the input image to output and draw the detected markers.
      outputImage = inputImage.clone();

      cout << "Drawing detected markers.\n";
      aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

      cout << "Estimating pose of the detected markers.\n";
      aruco::estimatePoseSingleMarkers(markerCorners, 0.053,
                                  cameraMatrix, distCoeffs, rvecs, tvecs);

      int i = 0;
      if (markerIds[i] == 0) {
        // drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        cout << "Marker " << markerIds[i] << ": " << endl;
        cout << "    Rotation: " << rvecs[i] << endl;
        cout << "    Translation: " << tvecs[i] << endl << endl;

        // Convert the rotation vector into a rotation matrix.
        cv::Rodrigues(rvecs[i], rotMatrix);

        // Concatenate the rotation matrix and transformed translation vector
        // to create the camera's transformation matrix to the world.
        hconcat(rotMatrix, -rotMatrix.t()*tvecs[i], intermediate);
        intermediate.convertTo(intermediate, CV_32F);
        vconcat(intermediate, padding, transCamToTag0);
        transCamToWorld = transTag0ToWorld*transCamToTag0;
        cout << "transCamToWorld: " << transCamToWorld << endl;

        // Update my pose information.
        my_pose.transform.translation.x = transCamToWorld.at<float>(0,3);
        my_pose.transform.translation.y = transCamToWorld.at<float>(1,3);
        my_pose.transform.translation.z = transCamToWorld.at<float>(2,3);

        // // Save the output image. Uncomment this for debugging.
        // imwrite(output_image_file, outputImage);
      }

      // Publish the updated pose.
      posePublisher.publish(&my_pose);
    }
    else
    {
      cout << "No Markers Detected!\n";
    }

    // Spin node and sleep for 1 second.
    nh.spinOnce();
    sleep(1);
  }

}