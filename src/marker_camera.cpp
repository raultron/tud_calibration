#include "tud_calibration/marker_camera_node.hpp"
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "marker_camera");  // Name of the node
  MarkerCameraNode Node;

   int32_t looprate = 100; //hz
   ros::Rate loop_rate(looprate);

  // ros::spin();
  while (Node.nh_.ok()) {
    ros::spinOnce();
    Node.find_marker_camera_tf();
    loop_rate.sleep();
  }
}
