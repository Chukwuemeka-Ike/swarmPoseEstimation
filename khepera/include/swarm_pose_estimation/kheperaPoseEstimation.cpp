#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <geometry_msgs/TransformStamped.h>

#include <stdlib.h>
#include <iostream>

#include <swarm_pose_estimation/kheperaPoseEstimation.h>

using namespace std;
using namespace cv;

/**
 * @brief Gets the locations of the tags in the world map from the file.
 * 
 * @param tagLocations map containing the tag numbers and their transformation
 *      matrix.
 * @param tagLocationsFile name of the YAML file with the tag locations.
*/
void getTagLocations(map<int, Mat> &tagLocations, string tagLocationsFile)
{    
    string tempString;

    // Open the tag locations file.
    FileStorage fs(tagLocationsFile, FileStorage::READ);
    if(!fs.isOpened())
    {
        cerr << "Unable to access tag locations!" << endl;
        return;
    }

    // Read the tagNames sequence.
    FileNode n = fs["tagNames"];                         
    if (n.type() != FileNode::SEQ)
    {
        cerr << "tagNames is not a sequence! FAIL" << endl;
        cerr << "tagNames type: " << n.type() << endl;
        cerr << "Sequence type: " << FileNode::SEQ << endl;
        return;
    }

    // Go through tagNames and add each tag number and location to the vectors.
    FileNodeIterator it = n.begin(), it_end = n.end(); 
    for (; it != it_end; ++it) 
    {
        Mat temp;
        tempString = (string)*it;
        fs["tagLocations"][tempString] >> temp;
        temp.convertTo(temp, CV_32F);

        // TODO: make this cleaner. back() only allows single-digit tags.
        tagLocations.insert(pair<int, Mat> (
          stoi(to_string(tempString.back())) - 48,
                                            temp));
    }
    fs.release();
}

/**
 * @brief Get the camera parameters from the camera data file.
 * 
 * @param cameraMatrix 
 * @param distCoeffs 
 * @param camera_params_file 
 */
void getCameraParams(Mat &cameraMatrix, Mat &distCoeffs, string camera_params_file)
{
  FileStorage fs(camera_params_file, FileStorage::READ);
  if(!fs.isOpened())
  {
    cerr << "Unable to access camera parameters!" << endl;
    return;
  }
  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;
  fs.release();
}

void captureInputImage(string capture_command, string capture_file, Mat &inputImage) 
{
  cout << "Taking Image\n";
  system(capture_command.c_str());

  cout << "Reading image into matrix.\n";
  inputImage = imread(capture_file);
}


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
) 
{
  // Perform detection on the input image.
  cout << "Detecting markers.\n";
  aruco::detectMarkers(inputImage, dictionary, markerCorners, 
                                markerIds, parameters, rejectedCandidates);

  // If at least one marker is detected, we can attempt to estimate
  // the robot's pose.
  if (markerIds.size() > 0) {
    // Copy the input image to output and draw the detected markers.
    outputImage = inputImage.clone();

    cout << "Drawing detected markers.\n";
    aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    cout << "Estimating pose of the detected markers.\n";
    aruco::estimatePoseSingleMarkers(markerCorners, 0.053,
                                cameraMatrix, distCoeffs, rvecs, tvecs);

    int i = 0;
    drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

    // Uncomment for debugging.
    // cout << "Marker " << markerIds[i] << ": " << endl;
    // cout << "    Rotation: " << rvecs[i] << endl;
    // cout << "    Translation: " << tvecs[i] << endl << endl;

    // Convert the rotation vector into a rotation matrix.
    cv::Rodrigues(rvecs[i], rotMatrix);

    // Concatenate the rotation matrix and transformed translation vector
    // to create the camera's transformation matrix to the world.
    hconcat(rotMatrix, -rotMatrix.t()*tvecs[i], intermediate);
    intermediate.convertTo(intermediate, CV_32F);
    vconcat(intermediate, padding, transCamToTag);
    transCamToWorld = tagLocations[markerIds[i]]*transCamToTag;

    // Uncomment for debugging.
    // cout << "transCamToWorld: " << transCamToWorld << endl;

    // Update my pose information.
    my_pose.transform.translation.x = transCamToWorld.at<float>(0,3);
    my_pose.transform.translation.y = transCamToWorld.at<float>(1,3);
    my_pose.transform.translation.z = transCamToWorld.at<float>(2,3);

    // Print out the robot's position.
    cout << "My position: \n";
    cout << "x: " << my_pose.transform.translation.x << endl;
    cout << "y: " << my_pose.transform.translation.y << endl;
    cout << "z: " << my_pose.transform.translation.z << endl << endl;

    // Save the output image. Uncomment this for debugging.
    // imwrite(output_image_file, outputImage);
  }
  else {
    cout << "No Markers Detected!\n";
  }
}
