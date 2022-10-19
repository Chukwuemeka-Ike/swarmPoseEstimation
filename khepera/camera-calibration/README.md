# Camera Calibration

To be able to use the Khepera's camera for pose estimation, the camera needs to be calibrated. The files contained in this folder are intended to streamline this process. [This link](https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html) explains the process of calibrating the camera with a detailed description of the C++ code. Here, I give a quick summary of the steps involved in calibration using the package.

## Usage
1. Print a [calibration chessboard](https://github.com/opencv/opencv/blob/4.x/doc/pattern.png) and attach it to a solid object you can move around easily - e.g back of a hardcover notebook. You can also keep the chessboard fixed, and move the Khepera around instead.
2. Edit the <Square_Size>, <BoardSize_Width>, <BoardSize_Height> entries in the *config.xml* file to match the dimensions of the chessboard you printed above.
3. Build the *cameraCalibration* binary file. 
    ```bash
    cd camera-calibration/
    cmake .
    make
    ```
4. The *CMakeLists.txt* file included in this folder generates the output in a *bin/* subfolder, so transfer the *cameraCalibration* binary to the *camera-calibration/* folder, then transfer the entire folder to the Khepera.
    ```bash
    cd camera-calibration/
    mv bin/cameraCalibration .
    scp -r ../camera-calibration/ root@<khepera_ip>:~/
    ```
5. On the Khepera, take calibration images with the camera by running the *takeCalibrationImages* bash script which takes 80 photos and saves them in an *images/* subfolder. As the script runs, rotate the chessboard through several different positions in order to give the calibration utility enough angles in the next step. Be sure to do this in a well-lit environment with the whole chessboard in the camera's field of view. [This video](https://www.youtube.com/watch?time_continue=38&v=yAYqt3RpT6c&feature=emb_logo) shows the image-capture process you should try to emulate albeit without the helpful GUI.
    ```bash
    cd camera-calibration/
    chmod +x takeCalibrationImages.bash
    ./takeCalibrationImages
    ```
6. Once the bash script is done taking images, run the *cameraCalibration* binary. This will use the settings in *config.xml* to calibrate the camera and write the camera parameters to the filename specified in its <Write_outputFileName> tag.
    ```bash
    ./cameraCalibration
    ```
    In any subsequent work that uses OpenCV for pose estimation, *posePublisher* for example, <Write_outputFileName> needs to be in the binary's working directory.


## Files
A quick description of the important files.


### takeCalibrationImages.bash
This bash script automates the image capture process by capturing 80 photos, 1 per second. The more photos are taken, the better the final calibration will be, but only up to a point (left vague here because I don't know what point). To calibrate the initial Khepera I had in my possession, I took 80 images, so that's what's set in the script. The resulting accuracy was enough for my work.


### imageList.xml
Having taken the images, it is then necessary to write an XML file which lists the location of the calibration images taken. A sample format of this list is available [here](https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/camera_calibration/VID5.xml). Assuming you did not edit the bash script above, the *imageList.xml* file already contains the correct images' location, so it can be used as is.


### config.xml
This file contains information necessary for calibrating the camera. An example is available [here](https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/camera_calibration/in_VID5.xml). The most important tags are the <Square_Size>, <BoardSize_Width>, <BoardSize_Height>, \<Input>, and <Write_outputFileName>. The comments should make it clear what each item is. Ensure that the \<Input> tag has the correct XML filename where the images are listed (*imageList.xml* here).


### cameraCalibration.cpp
This is the main program that calibrates the camera on the robot. It uses the settings in *config.xml* (hardcoded filename) to do so. If you change *config.xml*, you would have to edit the code. Otherwise, it should run without any problems.


### CMakeLists.txt
An example on how to compile the calibration code for the Khepera. Set the OpenCV directory to the location you built the OpenCV libraries, and the compilers to the Khepera cross-compilers to ensure everything runs smoothly.