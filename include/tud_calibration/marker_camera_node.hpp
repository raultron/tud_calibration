#ifndef MARKER_CAMERA_NODE_HPP
#define MARKER_CAMERA_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_srvs/Empty.h>
#include <angles/angles.h>

#include <boost/circular_buffer.hpp>

class MarkerCameraNode {
 public:
  MarkerCameraNode();

  ros::NodeHandle nh_;
  void find_marker_camera_tf(void);

 private:
  // Transformation broadcaster and listener
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;



};

#endif  // MARKER_CAMERA_NODE_HPP
