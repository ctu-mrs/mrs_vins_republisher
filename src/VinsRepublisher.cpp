/* includes //{ */

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/conversions.h>

#include <nodelet/nodelet.h>

#include <pluginlib/class_list_macros.h>

//}

namespace vins_republisher
{

/* class VinsRepublisher //{ */

class VinsRepublisher : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_        = false;
  bool _rate_limiter_enabled_ = false;

  /* ros parameters */
  std::string _uav_name_;
  bool        _rotate_velocity_ = false;
  /* std::string _camera_frame_; */
  std::string _fcu_frame_;
  std::string _mrs_vins_world_frame_;
  std::string _vins_fcu_frame_;
  double      _rate_limiter_rate_;
  bool        _init_in_zero_;

  bool   got_init_pose_ = false;
  double init_hdg_;

  bool validateOdometry(const nav_msgs::Odometry &odometry);

  // | ------------------------ callbacks ----------------------- |
  ros::Subscriber subscriber_vins_;
  void            odometryCallback(const nav_msgs::OdometryConstPtr &odom);

  ros::Publisher publisher_odom_;
  ros::Time      publisher_odom_last_published_;

  /* transformation handler */
  mrs_lib::Transformer transformer_;

  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;
};

//}

/* onInit() //{ */

void VinsRepublisher::onInit() {
  const std::string node_name("VinsRepublisher");

  /* obtain node handle */
  /* ros::NodeHandle nh("~"); */
  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[%s]: Initializing", node_name.c_str());

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  publisher_odom_last_published_ = ros::Time(0);

  // | ---------- loading ros parameters using mrs_lib ---------- |
  ROS_INFO("[%s]: loading parameters using ParamLoader", node_name.c_str());

  mrs_lib::ParamLoader param_loader(nh_, node_name);

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("rotate_velocity", _rotate_velocity_);
  param_loader.loadParam("rate_limiter/enabled", _rate_limiter_enabled_);
  param_loader.loadParam("rate_limiter/max_rate", _rate_limiter_rate_);
  if (_rate_limiter_rate_ <= 1e-3) {
    ROS_ERROR("[%s]: the rate limit has to be > 0", ros::this_node::getName().c_str());
    ros::shutdown();
  }

  param_loader.loadParam("fcu_frame", _fcu_frame_);
  /* param_loader.loadParam("camera_frame", _camera_frame_); */
  param_loader.loadParam("mrs_vins_world_frame", _mrs_vins_world_frame_);
  param_loader.loadParam("vins_fcu_frame", _vins_fcu_frame_);

  param_loader.loadParam("init_in_zero", _init_in_zero_, true);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: parameter loading failure", node_name.c_str());
    ros::shutdown();
  }

  /* transformation handler */
  transformer_ = mrs_lib::Transformer("VinsRepublisher");

  broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>();

  // | ----------------------- subscribers ---------------------- |

  subscriber_vins_ = nh_.subscribe("vins_odom_in", 10, &VinsRepublisher::odometryCallback, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_odom_ = nh_.advertise<nav_msgs::Odometry>("vins_odom_out", 10);

  is_initialized_ = true;

  ROS_INFO_ONCE("[%s]: initialized", node_name.c_str());
}
//}

/* validateOdometry() //{ */

bool VinsRepublisher::validateOdometry(const nav_msgs::Odometry &odometry) {

  // check position

  if (!std::isfinite(odometry.pose.pose.position.x)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.position.x'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.position.y)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.position.y'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.position.z)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.position.z'!!!");
    return false;
  }

  // check orientation

  if (!std::isfinite(odometry.pose.pose.orientation.x)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.orientation.x'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.orientation.y)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.orientation.y'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.orientation.z)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.orientation.z'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.orientation.w)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.orientation.w'!!!");
    return false;
  }

  // check if the quaternion is sound

  if (fabs(Eigen::Vector4d(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
                           odometry.pose.pose.orientation.w)
               .norm() -
           1.0) > 1e-2) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: orientation is not sound!!!");
    return false;
  }

  // check velocity

  if (!std::isfinite(odometry.twist.twist.linear.x)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.twist.twist.linear.x'!!!");
    return false;
  }

  if (!std::isfinite(odometry.twist.twist.linear.y)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.twist.twist.linear.y'!!!");
    return false;
  }

  if (!std::isfinite(odometry.twist.twist.linear.z)) {
    ROS_ERROR_THROTTLE(1.0, "[ControlManager]: NaN detected in variable 'odometry.twist.twist.linear.z'!!!");
    return false;
  }

  return true;
}

//}

/* odometryCallback() //{ */

void VinsRepublisher::odometryCallback(const nav_msgs::OdometryConstPtr &odom) {

  if (!is_initialized_) {
    return;
  }

  if (!validateOdometry(*odom)) {
    ROS_ERROR("[VinsRepublisher]: input odometry is not numerically valid");
    return;
  }

  ROS_DEBUG("[VinsRepublisher]: %d ", odom->header.seq);
  ROS_DEBUG("[VinsRepublisher]: %d ", odom->header.stamp.sec);
  ROS_DEBUG("[VinsRepublisher]: %d ", odom->header.stamp.nsec);

  ROS_DEBUG("[now]: %f", ros::Time::now().toSec());
  ROS_DEBUG("[last published]: %f", publisher_odom_last_published_.toSec());
  ROS_DEBUG("[fabs diff]: %f", fabs((ros::Time::now() - publisher_odom_last_published_).toSec()));
  ROS_DEBUG("[diff]: %f", (ros::Time::now() - publisher_odom_last_published_).toSec());
  ROS_DEBUG("[rate_limiter]: %f", 1.0 / _rate_limiter_rate_);
  if (_rate_limiter_enabled_ && fabs((ros::Time::now() - publisher_odom_last_published_).toSec()) < (1.0 / (_rate_limiter_rate_))) {
    ROS_DEBUG("[%s]: skipping over", ros::this_node::getName().c_str());
    return;
  }

  ROS_DEBUG("[%s]: publishing", ros::this_node::getName().c_str());

  nav_msgs::Odometry odom_transformed;
  odom_transformed.header          = odom->header;
  odom_transformed.header.frame_id = _mrs_vins_world_frame_;

  // Reference frames:
  // IMU = VINS body frame
  // FCU = MRS FCU frame (aligned with pixhawk IMU)
  // GLOBAL = VINS world frame
  // MRS = MRS world frame (created so that initial odometry heading is zero)
  //
  // VINS provides odometry corresponding to T^GLOBAL_IMU
  // We want to get odometry corresponding to T^MRS_FCU
  // So we do T^MRS_FCU = T^MRS_GLOBAL * T^GLOBAL_IMU * T^IMU_FCU
  //
  // (T^A_B represents transformation that transforms points from B to A)

  // T^IMU_FCU - transforms points from FCU to IMU
  auto T_IMU_FCU = transformer_.getTransform(_fcu_frame_, _vins_fcu_frame_, odom->header.stamp);
  if (!T_IMU_FCU) {
    ROS_WARN_THROTTLE(1.0, "[%s]: could not find transform from '%s' to '%s'", ros::this_node::getName().c_str(), _fcu_frame_.c_str(),
                      _vins_fcu_frame_.c_str());
    return;
  }

  /* transform pose *//*//{*/
  geometry_msgs::Pose pose_transformed;

  // R^IMU_FCU
  Eigen::Matrix3d R_IMU_FCU = mrs_lib::AttitudeConverter(T_IMU_FCU.value().transform.rotation);

  // R^GLOBAL_IMU = vio odometry orientation
  Eigen::Matrix3d R_GLOBAL_IMU = mrs_lib::AttitudeConverter(odom->pose.pose.orientation);

  // R^GLOBAL_FCU = R^GLOBAL_IMU * R^IMU_FCU
  pose_transformed.orientation = mrs_lib::AttitudeConverter(R_GLOBAL_IMU * R_IMU_FCU);

  // t^IMU_FCU
  Eigen::Vector3d t_IMU_FCU;
  t_IMU_FCU << T_IMU_FCU.value().transform.translation.x, T_IMU_FCU.value().transform.translation.y, T_IMU_FCU.value().transform.translation.z;

  // t^GLOBAL_FCU = R^GLOBAL_IMU * t^IMU_FCU + t^GLOBAL_IMU
  Eigen::Vector3d translation = R_GLOBAL_IMU * t_IMU_FCU;
  pose_transformed.position.x = translation(0) + odom->pose.pose.position.x;
  pose_transformed.position.y = translation(1) + odom->pose.pose.position.y;
  pose_transformed.position.z = translation(2) + odom->pose.pose.position.z;

  // pose_transformed is now T^GLOBAL_FCU

  // then subtract initial pose (global 4 DOF)
  // save initial pose to subtract it from all messages to compensate initialized orientation ambiguity
  if (_init_in_zero_) {
    if (!got_init_pose_) {
      init_hdg_ = mrs_lib::AttitudeConverter(pose_transformed.orientation).getHeading();
      ROS_INFO("[VinsRepublisher]: init hdg: %.2f", init_hdg_);
      got_init_pose_ = true;
    }

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp            = odom->header.stamp;
    tf_msg.header.frame_id         = odom_transformed.header.frame_id;
    tf_msg.child_frame_id          = odom->header.frame_id;
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.rotation      = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(-init_hdg_);

    // obtain T^MRS_FCU
    tf2::doTransform(pose_transformed, pose_transformed, tf_msg);

    Eigen::Matrix3d R_MRS_GLOBAL = mrs_lib::AttitudeConverter(tf_msg.transform.rotation);

    /* Eigen::Matrix3d R_MRS_GLOBAL2; */
    /* R_MRS_GLOBAL2 << cos(-init_hdg_), -sin(-init_hdg_), 0, sin(-init_hdg_), cos(-init_hdg_), 0, 0, 0, 1; */

  }

  // publish the initial offset to TF - T^GLOBAL_MRS
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp    = odom->header.stamp;
  tf_msg.header.frame_id = odom->header.frame_id;
  tf_msg.child_frame_id  = odom_transformed.header.frame_id;
  tf_msg.transform.translation.x = 0;
  tf_msg.transform.translation.x = 0;
  tf_msg.transform.translation.x = 0;
  tf_msg.transform.rotation      = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(init_hdg_);

  try {
    ROS_INFO_THROTTLE(1.0, "[UavPoseEstimator]: Publishing TF.");
    broadcaster_->sendTransform(tf_msg);
  }
  catch (...) {
    ROS_ERROR("[UavPoseEstimator]: Exception caught during publishing TF: %s - %s.", tf_msg.child_frame_id.c_str(), tf_msg.header.frame_id.c_str());
  }

  odom_transformed.pose.pose = pose_transformed;
/*//}*/

  /* transform velocity - linear and angular *//*//{*/
  // if in body frame - rotate from IMU frame to FCU frame
  geometry_msgs::Vector3 linear_velocity = odom->twist.twist.linear;
  Eigen::Vector3d v2;
  v2 << linear_velocity.x, linear_velocity.y, linear_velocity.z;
  v2                = R_IMU_FCU.transpose() * v2;
  linear_velocity.x = v2(0);
  linear_velocity.y = v2(1);
  linear_velocity.z = v2(2);

  geometry_msgs::Vector3 angular_velocity = odom->twist.twist.angular;
  Eigen::Vector3d v3;
  v3 << angular_velocity.x, angular_velocity.y, angular_velocity.z;
  v3                 = R_IMU_FCU.transpose() * v3;
  angular_velocity.x = v3(0);
  angular_velocity.y = v3(1);
  angular_velocity.z = v3(2);

  // TODO if in global frame, apply initial TF offset

  odom_transformed.twist.twist.linear  = linear_velocity;
  odom_transformed.twist.twist.angular = angular_velocity;
/*//}*/

  // transform covariance - apparently translation is in world frame and rotation is in body frame with fixed axes???
  // if so, apply initial TF offset to translation part and IMU to FCU TF to rotation part??? + twist covariance...


  // publish
  if (!validateOdometry(odom_transformed)) {
    ROS_ERROR("[VinsRepublisher]: transformed odometry is not numerically valid");
    return;
  }

  try {
    publisher_odom_.publish(odom_transformed);
    ROS_INFO_THROTTLE(1.0, "[%s]: Publishing", ros::this_node::getName().c_str());
    publisher_odom_last_published_ = ros::Time::now();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_odom_.getTopic().c_str());
  }

}

//}

}  // namespace vins_republisher
/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(vins_republisher::VinsRepublisher, nodelet::Nodelet);
