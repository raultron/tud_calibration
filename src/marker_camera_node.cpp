#include "tud_calibration/marker_camera_node.hpp"

MarkerCameraNode::MarkerCameraNode() {
  ros::NodeHandle params("~");

  //params.param<std::string>("cmd_vel_ref_topic", s, "/cmd_vel_ref");
  marker_to_cam_buffer.set_capacity(200);

}

void MarkerCameraNode::find_marker_camera_tf(void){
  tf::StampedTransform calibExt_to_marker;
  tf::StampedTransform cam_to_calib;
  tf::StampedTransform calib_to_calibExt;
  tf::StampedTransform marker_to_cam;

  ros::Time now = ros::Time::now();

  //! calibration_marker_ext and calibration_marker are the same frame
  //! We do the separation since we can't have a TF tree with a node
  //! that has two parents (in this case the two parents are the two cameras)
  calib_to_calibExt.frame_id_ = "calibration_marker";
  calib_to_calibExt.child_frame_id_ = "calibration_marker_ext";
  calib_to_calibExt.setIdentity();

  // From external camera we expect: Main Marker ---> Calibration Marker
  try {
    tf_listener_.waitForTransform("calibration_marker_ext", "marker", ros::Time(0),
                                  ros::Duration(5));
    tf_listener_.lookupTransform("calibration_marker_ext", "marker", ros::Time(0),
                                 calibExt_to_marker);
    // ROS_DEBUG("tracking_node | Parent Frame: %s, Child frame: %s",
    // base_to_target.frame_id_.c_str(),
    // base_to_target.child_frame_id_.c_str());
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // From the robot camera we expect: Cam ---> Calibration Marker
  //!TODO(racuna) replace the names using arguments
  //! Right now I now that vrmagic optical frame is the lef camera of the stereo camera
  //! so we are going to perform the calibration related to this frame
  try {
    tf_listener_.waitForTransform("vrmagic", "calibration_marker", ros::Time(0),
                                  ros::Duration(5));
    tf_listener_.lookupTransform("vrmagic", "calibration_marker", ros::Time(0),
                                 cam_to_calib);
    // ROS_DEBUG("tracking_node | Parent Frame: %s, Child frame: %s",
    // base_to_target.frame_id_.c_str(),
    // base_to_target.child_frame_id_.c_str());
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Transform marker_to_cam_tf;

  marker_to_cam_tf = (cam_to_calib*calib_to_calibExt*calibExt_to_marker).inverse();

  marker_to_cam.setData(marker_to_cam_tf);
  marker_to_cam.frame_id_ = "marker";
  marker_to_cam.child_frame_id_ = "vrmagic";
  marker_to_cam.stamp_ = ros::Time::now();  

  marker_to_cam_buffer.push_back(marker_to_cam);

  marker_to_cam_tf = do_interpolation_tf_buffer(marker_to_cam_buffer);
  marker_to_cam.setData(marker_to_cam_tf);


  tf_broadcaster_.sendTransform(marker_to_cam);
  ROS_INFO("Position: (%f ; %f ; %f ", marker_to_cam_tf.getOrigin().x(), marker_to_cam_tf.getOrigin().y(), marker_to_cam_tf.getOrigin().z());
  ROS_INFO("Orientation: (%f ; %f ; %f ; %f", marker_to_cam_tf.getRotation().x(), marker_to_cam_tf.getRotation().y(), marker_to_cam_tf.getRotation().z(),  marker_to_cam_tf.getRotation().w());
}



tf::StampedTransform& MarkerCameraNode::tf_interpolation(tf::StampedTransform& new_tf,
                                           tf::StampedTransform& old_tf, double interpolation_weight) {

  tf::Vector3 pos = vector_interpolation(new_tf.getOrigin(), old_tf.getOrigin(), interpolation_weight);
  new_tf.setOrigin(pos);

  tf::Quaternion orn = quaternion_interpolation(new_tf.getRotation(), old_tf.getRotation(), interpolation_weight);
  new_tf.setRotation(orn);
  return new_tf;
}

tf::Vector3 MarkerCameraNode::vector_interpolation(
    tf::Vector3 new_vector, tf::Vector3 old_vector,
    double interpolation_weight) {
  tf::Vector3 interpolated_vector((1 - interpolation_weight) * old_vector.x() +
                                       interpolation_weight * new_vector.x(),
                                  (1 - interpolation_weight) * old_vector.y() +
                                       interpolation_weight * new_vector.y(),
                                  (1 - interpolation_weight) * old_vector.z() +
                                       interpolation_weight * new_vector.z());
  return interpolated_vector;
}

tf::Quaternion MarkerCameraNode::quaternion_interpolation(
    tf::Quaternion new_q, tf::Quaternion old_q,
    double interpolation_weight){
    tf::Quaternion interpolated_q;
    interpolated_q = old_q.slerp(new_q, interpolation_weight);
    return interpolated_q;
  }

tf::StampedTransform MarkerCameraNode::do_interpolation_tf_buffer(boost::circular_buffer<tf::StampedTransform> buffer){

  tf::StampedTransform tf_filtered;
  tf::StampedTransform tf_input = buffer[0];

  int count = 1.0;
  tf_filtered.setIdentity();

  for(tf::StampedTransform it : buffer){

    tf_filtered = tf_interpolation( it, tf_filtered, (1.0/count));
    count++;
    //ROS_INFO("X in each tf: %f, filtered: %f, count %d", it.getOrigin().x(), tf_filtered.getOrigin().x(), count);
  }

  tf_filtered.frame_id_ = tf_input.frame_id_;
  tf_filtered.child_frame_id_ = tf_input.child_frame_id_;
  tf_filtered.stamp_ = ros::Time::now();

  return tf_filtered;
}

