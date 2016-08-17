#include "tud_calibration/marker_camera_node.hpp"

MarkerCameraNode::MarkerCameraNode() {
  ros::NodeHandle params("~");

  //params.param<std::string>("cmd_vel_ref_topic", s, "/cmd_vel_ref");

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

  marker_to_cam.setData(cam_to_calib*calib_to_calibExt*calibExt_to_marker);
  marker_to_cam.frame_id_ = "marker";
  marker_to_cam.child_frame_id_ = "vrmagic";
  marker_to_cam.stamp_ = ros::Time::now();

  tf_broadcaster_.sendTransform(marker_to_cam);
}
