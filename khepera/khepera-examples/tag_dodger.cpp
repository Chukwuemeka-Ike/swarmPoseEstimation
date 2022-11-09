/**
 * Takes a photo each second, looks for tags, turns 90 degrees if any visible.
 * 
 * This program uses OpenCV to continuously take photos and scan each
 * for Aruco tags. If any tags are visible, the Khepera rotates 90 degrees.
 * It does this until Ctrl-c is pressed.
 * 
 * Usage:
 *  ./tagDodger
*/
#include <khepera/khepera.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <signal.h>
#include <stdlib.h>
#include <iostream>

using namespace std;
using namespace cv;

static knet_dev_t * dsPic; // robot pic microcontroller access
static int quitReq = 0; // quit variable for loop

static void ctrlc_handler(int sig);
void getCameraParams(Mat &cameraMatrix, Mat &distCoeffs, string camera_params_file);
void captureInputImage(string capture_command, string capture_file, Mat &inputImage);
bool areTagsPresent(
  Mat &inputImage, Mat &outputImage, Mat &cameraMatrix, Mat &distCoeffs,
  vector<Vec3d> &rvecs, vector<Vec3d> &tvecs,
  vector<int> &markerIds, Ptr<aruco::Dictionary> &dictionary,
  Ptr<aruco::DetectorParameters> &parameters,
  vector<vector<Point2f> > &markerCorners,
  vector<vector<Point2f> > &rejectedCandidates,
  string output_image_file
);
void rotateKhepera(knet_dev_t * dsPic, int &lpos, int &rpos);


int main(int argc, char **argv) {
  // Motor control and position variables.
  int lpos, rpos;
  int kp,ki,kd;
  int pmarg,maxsp,accinc,accdiv,minspacc, minspdec; // Speed profiles 

  bool tagsPresent;

  /* Set the libkhepera debug level - Highly recommended for development. */
  kb_set_debug_level(2);

  // File names and camera capture command.
  string capture_file = "input_img.jpg";
  string output_image_file = "output_img.jpg";
  string camera_params_file = "cameraData.xml";
  string capture_command = "v4l2grab -d /dev/video6 -o "
                            + capture_file + " -W 752 -H 480 -q 85 -I -1";


  // OpenCV and ArUco variables for images, camera parameters, and pose info.
  Mat inputImage, outputImage, cameraMatrix, distCoeffs;
  vector<int> markerIds;
  vector<vector<Point2f> > markerCorners, rejectedCandidates;
  vector<Vec3d> rvecs, tvecs;
  Ptr<aruco::DetectorParameters> parameters =
                                      aruco::DetectorParameters::create();
  Ptr<aruco::Dictionary> dictionary =
                    aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

  // Initialize libkhepera and robot access.
  if ( kh4_init(0,NULL)!=0) {
  	printf("\nERROR: could not initiate the libkhepera!\n\n");
  	return -1;
  }	

  // Open the robot socket and store the handle in a pointer.
  dsPic  = knet_open( "Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);

  if (dsPic == NULL) {
  	printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
  	return -2;
  }	

  // Initialize the motor controllers.
  // Tuned motor parameters.
  pmarg = 20;
  kh4_SetPositionMargin(pmarg, dsPic); 				// position control margin

  // Configure the PID controller. Not useful for non error-based control.
  kp = 10;
  ki = 5;
  kd = 1;
  kh4_ConfigurePID( kp, ki, kd, dsPic); 		
  
  // Acceleration increment,  Acceleration divider, Minimum speed acc,
  // Minimum speed dec, maximum speed.
  accinc = 3;
  accdiv = 0;
  minspacc = 20;
  minspdec = 1;
  maxsp = 400;
  kh4_SetSpeedProfile(accinc, accdiv, minspacc, minspdec, maxsp, dsPic);
  
  // Put in idle mode (no control).
	kh4_SetMode(kh4RegIdle, dsPic);

  // Set the signal for catching ctrl-c.
  signal( SIGINT , ctrlc_handler ); 

  // Get the camera parameters.
  getCameraParams(cameraMatrix, distCoeffs, camera_params_file);

  // Set up camera format.
  cout << "Setting Pipes\n";
  system("media-pipes.sh");
  cout << "Setting Format\n";
  system("media-formats.sh 752 480");

  // Loop until ctrl-c key.
  while(quitReq == 0)
  {
    captureInputImage(capture_command, capture_file, inputImage);
    tagsPresent = areTagsPresent(
      inputImage, outputImage, cameraMatrix, distCoeffs,
      rvecs, tvecs, markerIds, dictionary, parameters,
      markerCorners, rejectedCandidates,
      output_image_file
    );

    if (tagsPresent) {
      rotateKhepera(dsPic, lpos, rpos);
    }   
  }
}

/**
 * @brief Ends the program properly when ctrl-c is pressed.
 * 
 * @param sig 
 */
static void ctrlc_handler( int sig ) 
{
  quitReq = 1;
  
  // Stop the robot and set it to idle mode.
  kh4_set_speed(0, 0, dsPic);
  kh4_SetMode(kh4RegIdle, dsPic);
  
  // Clear RGB LEDs.
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0, dsPic); 
  
  // Revert to the original terminal if it was changed.
  kb_change_term_mode(0);
  
  exit(0);
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

/**
 * @brief Capture the input image.
 * 
 * @param capture_command 
 * @param capture_file 
 * @param inputImage 
 */
void captureInputImage(string capture_command, string capture_file, Mat &inputImage) 
{
  cout << "Taking Image\n";
  system(capture_command.c_str());

  cout << "Reading image into matrix.\n";
  inputImage = imread(capture_file);
}

/**
 * Checks if any tags are present in the input image.
*/
bool areTagsPresent(
  Mat &inputImage, Mat &outputImage, Mat &cameraMatrix, Mat &distCoeffs,
  vector<Vec3d> &rvecs, vector<Vec3d> &tvecs,
  vector<int> &markerIds, Ptr<aruco::Dictionary> &dictionary,
  Ptr<aruco::DetectorParameters> &parameters,
  vector<vector<Point2f> > &markerCorners,
  vector<vector<Point2f> > &rejectedCandidates,
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

    // Save the output image. Uncomment this for debugging.
    imwrite(output_image_file, outputImage);
    return true;
  }
  else {
    return false;
  }
  
}

/**
 * @brief Rotates the Khepera by 90-ish degrees each time it sees a tag.
 * 
 * @param dsPic - pointer to the microcontroller.
 * @param lpos - left motor's position.
 * @param rpos - right motor's position.
 */
void rotateKhepera(knet_dev_t * dsPic, int &lpos, int &rpos) {
  long motspeed;

  // Get the current motor position.
  kh4_get_position(&lpos, &rpos, dsPic);
  printf("\nMotor positions: left %ld | right %ld\n", lpos, rpos);

  // Set control mode to speed regulation, then set the speed and wait n seconds.
  kh4_SetMode(kh4RegSpeed, dsPic);
  long speed = 80.0;
  motspeed = (long)(speed/KH4_SPEED_TO_MM_S);
  kh4_set_speed(motspeed, -motspeed, dsPic);
  printf("\nRotating 2.25s at %.1f mm/s (pulse speed %ld) with speed only\n", speed, motspeed);
  sleep(1.125);

  // Stop rotating, then check new position.
  kh4_set_speed(0 , 0, dsPic);
  kh4_get_position(&lpos, &rpos, dsPic);
  printf("\nMotor positions: left %ld | right %ld\n", lpos, rpos);
}