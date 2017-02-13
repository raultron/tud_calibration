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

  tf::StampedTransform& tf_interpolation(tf::StampedTransform& new_tf, tf::StampedTransform& old_tf, double interpolation_weight);
  tf::Vector3 vector_interpolation(tf::Vector3 new_vector, tf::Vector3 old_vector, double interpolation_weight);
  tf::Quaternion quaternion_interpolation(tf::Quaternion new_q, tf::Quaternion old_q, double interpolation_weight);
  tf::StampedTransform do_interpolation_tf_buffer(boost::circular_buffer<tf::StampedTransform> buffer);


  boost::circular_buffer<tf::StampedTransform> marker_to_cam_buffer;



};

#endif  // MARKER_CAMERA_NODE_HPP
