#ifndef KHEPERA_POSE_ESTIMATION_H
#define KHEPERA_POSE_ESTIMATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <stdlib.h>
#include <iostream>

using namespace std;
using namespace cv;

void getTagLocations(map<int, Mat> &tagLocations, string tagLocationsFile);
void getCameraParams(Mat &cameraMatrix, Mat &distCoeffs, string camera_params_file);
void captureInputImage(string capture_command, string capture_file, Mat &inputImage);
void estimateKheperaPose(
  Mat &inputImage, Mat &outputImage,
  Mat &cameraMatrix, Mat &distCoeffs,
  Mat &rotMatrix, Mat &intermediate, Mat &padding,
  Mat &transCamToTag, Mat &transCamToWorld,
  vector<Vec3d> &rvecs, vector<Vec3d> &tvecs,
  vector<int> &markerIds,
  Ptr<aruco::Dictionary> &dictionary,
  Ptr<aruco::DetectorParameters> &parameters,
  vector<vector<Point2f> > &markerCorners,
  vector<vector<Point2f> > &rejectedCandidates,
  map<int, Mat> &tagLocations, geometry_msgs::TransformStamped &my_pose,
  string output_image_file
);

#endif