# Khepera Swarm Controller

This repository contains packages for controlling a swarm of Khepera robots using a central ROS host machine. The code is organized into two major components:

1. host - contains packages for the ROS host machine, and
2. khepera - contains packages for each Khepera in the swarm.

This README is written so the user can start with a fresh Windows 11 system and install all components necessary to develop additional packages, or run the pre-built binaries.


## Contents
* [Usage](#usage)
* [Windows Subsystem for Linux](#windows-subsystem-for-linux)
* [ROS Setup](#ros-setup)
* [Khepera Setup](#khepera-setup)
* [Development](#development)

## Usage
This section describes how to use the code in this repository assuming you have a functioning Linux, ROS, rosserial, and Khepera system, along with a functioning network connection. If not, please follow the setup sections below before continuing this section. Using the project involves running code on each Khepera in the swarm and on the ROS host, detailed below.


### Host
To build the *swarm_pose_estimator* package and make it available to ROS, run the commands
```bash
cd host
catkin_make
source host/devel/setup.bash
```
Once these are done, we need to start roscore and a rosserial_python node to listen for Khepera communications. These can be done by running the following commands in separate terminals
```bash
roscore
rosrun rosserial_python serial_node.py tcp
```
The rosserial_python package takes care of TCP connections with each of the Kheperas in the swarm. Once both are running, you can run the *swarm_pose_estimator*'s *centroid_estimator* node. It subscribes to the *khepera_pose* ROS topic, aggregates the swarm's pose, and publishes the centroid on the *swarm_centroid* topic.
```bash
rosrun swarm_pose_estimator centroid_estimator <num_kheperas>
```
**Alternatively**, you can use the [launch file](https://github.com/Chukwuemeka-Ike/swarmPoseEstimation/blob/master/host/src/centroid_estimator.launch) I already wrote to combine all of these steps into one.
```bash
roslaunch swarm_pose_estimator centroid_estimator.launch swarm_size:=<num_kheperas>
```
<!-- http://wiki.ros.org/roslaunch/XML.  -->


### Khepera
In this section, I assume you've already calibrated the camera. If not, the [*khepera/camera-calibration*](https://github.com/Chukwuemeka-Ike/swarmPoseEstimation/blob/master/khepera/camera-calibration) folder has instructions on doing so. To build the pose-publisher binary, navigate to the *khepera/pose-publisher* folder and build the targets with the following commands
```bash
cd khepera/pose-publisher
cmake . && make
```
The built binary will be in the *bin* subdirectory. Copy the binary to each Khepera with the command
```bash
scp bin/<pose_publisher_binary> root@<khepera_ip>:~/<pose_publisher_dir>
```
On each Khepera, run the binary with the command
```bash
cd ~/<pose_publisher_dir>
./<pose_publisher_binary> <host_ip>
```

With all components running, the Kheperas should now be publishing their position based on the tags they see, and the host machine should be calculating the swarm's centroid based on that information.



## Windows Subsystem for Linux
To avoid using a virtual machine or a dual boot Linux system, I installed the [Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/about) (WSL) on my computer. As I understand it, it's a compatibility layer for natively running Linux binaries in the Windows 10 console. It's a great way to avoid the more complex aspects of dual booting, but it comes with some of its own added wrinkles - most notably networking for which I detail a workaround.

I used the installation instructions available from Microsoft [here](https://docs.microsoft.com/en-us/windows/wsl/install) to install Ubuntu 20.04.


### Port Proxy Networking
Rosserial (discussed below) needs access to a TCP port on the host machine, but because I'm using WSL, I found that I couldn't directly reach the Linux system from the local network. To bypass this, I used a PowerShell script based on [this post](https://dev.to/vishnumohanrk/wsl-port-forwarding-2e22) and [this post](https://github.com/microsoft/WSL/issues/4150#issuecomment-504209723) to forward the specific TCP port I needed to the Linux system. [This post](https://learn.microsoft.com/en-us/windows/wsl/networking) explains why this is necessary, but following it directly did not work for me.

The *port_forwarding.ps1* script is included in this repo to streamline your efforts. Note that it cannot be run if the script is on the Linux system due to Powershell's security policies. Copy it to your Windows' drive before attempting to run it.



## Khepera Setup
We chose the Khepera IV mobile robot as our robotics platform. [This repository](https://github.com/Chukwuemeka-Ike/kheperaIVPoseEstimation) talks about some features of the robot and setting up a development environment on an Ubuntu system. 


### WiFi Connection
Connecting the Khepera to a network is a bit tricky, so this section discusses easily setting it up. I connected the Khepera to a WiFi network with WPA2-Personal encryption using the following steps:
1. Run the command below
```bash
wpa_passphrase My_SSID My_Passphrase
```
which generates an output similar to
```bash
network={
        ssid="My_SSID"
        #psk="My_Passphrase"
        psk=My_PSK
}
```

2. Copy the output of the above wpa_passphrase command into the file at */etc/wpa_supplicant/wpa_supplicant-\<interface>.conf* where \<interface> is the device you'd like to use. In my case, I was using *wlan0*.
3. I wasn't able to figure out automatic ip assignment, so I opted to use a static IP address. To do so, I edited the file at */etc/systemd/network/wifi.network*
```
[Match]
Name=<interface>

[Network]
#DHCP=v4
Address=<IP Address>/<netmask>

[DHCP]
RouteMetric=20
```
4. Reboot the Khepera
```bash
reboot
```
5. Run the following command:
```bash
ip a show <interface>
```
Crosscheck that the address following *inet* matches what was specified in *wifi.network*. If it is, then you're good to go.


 
## ROS Setup
Once I had WSL installed, I went on to install the Robot Operating System (ROS). Since WSL feels like a native Linux environment, I was able to follow the steps from the ROS wiki [here](https://wiki.ros.org/noetic/Installation/Ubuntu). In this project, I'm using ROS Noetic, which is the last ROS 1 distro planned. The guide linked above is for installing Noetic on an Ubuntu 20.04 system.


### ROS Serial Embedded Linux
The Khepera cannot run a fully fledged ROS distribution, so we had to use a serial package designed to bring elements of ROS functionality to embedded systems. With the ROS Host setup, the next step is to build the [rosserial_embeddedlinux](http://wiki.ros.org/rosserial_embeddedlinux) package for the Host and the Khepera. I installed Rosserial from Source based on [these instructions](http://wiki.ros.org/rosserial_embeddedlinux/GenericInstall), which I've repeated here:

1. Create a catkin workspace 
2. Clone the [rosserial repository](https://github.com/ros-drivers/rosserial.git) into the *src* subdirectory
3. Catkin make and install the package
4. Source the setup file
```bash
cd ~
mkdir <ws> && mkdir <ws>/src
cd <ws>/src
git clone https://github.com/ros-drivers/rosserial.git
cd <ws>
catkin_make
catkin_make install
```
The commands above generate the *rosserial_msgs* and build the *ros_lib* library in the \<ws>/install directory. To make the *rosserial* packages available to ROS, run the following command
```bash
source <ws>/install/setup.bash
```
This has to be run in every terminal you use, so I suggest adding the command to *~/.bashrc* to automate the process. Once the packages have been built, select a directory to hold the library files for the embedded system:
```bash
cd <some_directory>
rm -rf ros_lib examples
rosrun rosserial_embeddedlinux make_libraries.py
```
Add the *ros_lib/* folder contents to the include path for the cross-compiler you're using for the project. In my case, I just copied its contents directly into my package's *include/* folder and got the desired result.



## Development
This section describes how to develop code for the Host and the Khepera.

### Host Development
All the Host packages were developed for ROS with the catkin build system, so assuming you've set up ROS properly, the following should be immediately useful. To build a simple ROS package, we need to first create a catkin workspace directory to hold the package. This involves creating a directory and then a *src* subdirectory.
```bash
mkdir <ws> && cd <ws>
mkdir src
```
From within the *src* subdirectory, run the catkin command to create a package with your desired name. 
```bash
cd src
catkin_create_pkg <package_name>
cd <package_dir>
```
This command creates a *package.xml* and *CMakeLists.txt* in the package directory, which are vital for documenting the package and directing the compiler in how to build any source code you write, respectively. Once you've written code for your package, you can use catkin to build every package in your workspace. The command must be run from the workspace root directory.
```bash
cd <ws>
catkin_make
```
If the command runs successfully, catkin will create two new folders in the workspace called *build/* and *devel/*. To make the packages in the workspace accessible to your system-wide ROS, run the command 
```bash
source <ws>/devel/setup.bash
```
[This link](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) describes creating and customizing ROS packages in greater detail. 


### Khepera Development
All of the code written for the Khepera in this project is in C++. I used [CMake](https://cmake.org/) to build each program with the Poky toolchain we set up previously.
