// #include <khepera.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>
// #include <stdio.h>
// #include <stdlib.h>
// #include <iostream>
// #include <time.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <stdio.h>

// // The two namespaces used in this code
// using namespace std;
// using namespace cv;

// My IP address
#define MY_IP "192.168.0.230"

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

// void messageCb (const std_msgs::String &received_msg){
//   printf("Received subscribed chatter message: %s\n", received_msg.data);
// }
// ros::Subscriber<std_msgs::String> sub("chatter", messageCb);

char rosSrvrIp[14] = "192.168.0.216";
char hello[13] = "Hello ROS!";

int main(int argc, char *argv[])
{
  int rc;

  //nh.initNode();
  nh.initNode(rosSrvrIp);

  // Example Publisher
  nh.advertise(chatter);
  while(1) {
    str_msg.data = hello;
    chatter.publish(&str_msg);
    nh.spinOnce();
    printf("chattered\n");
    sleep(1);
  }

  // // Example Subscriber
  // nh.subscribe(sub);
  // while(1) {
  //   nh.spinOnce();
  //   sleep(1);
  // }
}