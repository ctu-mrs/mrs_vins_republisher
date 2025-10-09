/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/conversions.h>

#include "log_rate_limiter.h"

//}

namespace vins_republisher
{

/* class VinsRepublisher //{ */

class VinsRepublisher : public rclcpp::Node {
public:
  //virtual void onInit();
  VinsRepublisher(const rclcpp::NodeOptions & options);

private:
  /* timer initialization */
  rclcpp::TimerBase::SharedPtr timer_initialization_;
  void timerInitialization();

  /* flags */
  bool is_initialized_        = false;
  bool _rate_limiter_enabled_ = false;

  /* ros parameters */
  std::string _uav_name_;
  bool        _velocity_in_body_frame_ = true;
  std::string _fcu_frame_;
  std::string _mrs_vins_world_frame_;
  std::string _vins_fcu_frame_;
  double      _rate_limiter_rate_;
  bool        _init_in_zero_;

  bool   got_init_pose_ = false;
  double init_hdg_;

  bool                                                validateOdometry(const nav_msgs::msg::Odometry &odometry);
  geometry_msgs::msg::PoseWithCovariance::_covariance_type transformCovariance(const geometry_msgs::msg::PoseWithCovariance::_covariance_type &cov_in,
                                                                          const tf2::Transform &                                     transform);

  // | ------------------------ callbacks ----------------------- |
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_vins_;
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr &odom);
  bool is_averaging_ = false;
  bool is_averaging_finished_ = false;
  int  n_imu_meas_ = 0;
  Eigen::Vector3d mean_acc_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_status_;
  rclcpp::Time publisher_odom_last_published_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srvs_calibrate_;
  bool compensate_initial_tilt_;
  bool calibrateSrvCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  bool is_calibrated_  = false;
  bool has_valid_odom_ = false;
  nav_msgs::msg::Odometry odom_init_;
  std::mutex mtx_odom_init_;

  /* transformation handler */
  mrs_lib::Transformer transformer_;

  std::shared_ptr<mrs_lib::TransformBroadcaster> broadcaster_;

  Eigen::Matrix3d Exp(const Eigen::Vector3d &ang);
  Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d &vec);
};

//}

/* onInit() //{ */

VinsRepublisher::VinsRepublisher(const rclcpp::NodeOptions & options) : rclcpp::Node("VinsRepublisher", options) {
  timer_initialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&VinsRepublisher::timerInitialization, this));
}

void VinsRepublisher::timerInitialization(){
  RCLCPP_INFO(get_logger(), "[%s]: Initializing", get_name());

  /* waits for the ROS to publish clock */
  //rclcpp::Time::waitForValid(); TODO: replacement?

  publisher_odom_last_published_ = get_clock()->now();

  mean_acc_ << 0, 0, 0;

  // | ---------- loading ros parameters using mrs_lib ---------- |
  RCLCPP_INFO(get_logger(), "[%s]: loading parameters using ParamLoader", get_name());

  mrs_lib::ParamLoader param_loader(shared_from_this(), get_name());

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    RCLCPP_INFO(get_logger(), "loading custom config '%s", custom_config_path.c_str());
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("velocity_in_body_frame", _velocity_in_body_frame_);
  param_loader.loadParam("rate_limiter/enabled", _rate_limiter_enabled_);
  param_loader.loadParam("rate_limiter/max_rate", _rate_limiter_rate_);
  if (_rate_limiter_rate_ <= 1e-3) {
    RCLCPP_ERROR(get_logger(), "[%s]: the rate limit has to be > 0", get_name());
    rclcpp::shutdown();
  }

  param_loader.loadParam("fcu_frame", _fcu_frame_);
  param_loader.loadParam("mrs_vins_world_frame", _mrs_vins_world_frame_);
  param_loader.loadParam("vins_fcu_frame", _vins_fcu_frame_);

  param_loader.loadParam("init_in_zero", _init_in_zero_, true);

  param_loader.loadParam("compensate_initial_tilt", compensate_initial_tilt_, false);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(get_logger(), "[%s]: parameter loading failure", get_name());
    rclcpp::shutdown();
  }

  /* transformation handler */
  transformer_ = mrs_lib::Transformer(shared_from_this());

  broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>(shared_from_this());

  // | ----------------------- subscribers ---------------------- |

  subscriber_vins_ = create_subscription<nav_msgs::msg::Odometry>("~/odom_in", 10, std::bind(&VinsRepublisher::odometryCallback, this, std::placeholders::_1));

  // | ----------------------- publishers ----------------------- |

  publisher_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/odom_out", 10);
  publisher_status_ = create_publisher<std_msgs::msg::String>("~/status_string_out", 2);

  if (compensate_initial_tilt_) {
    //srvs_calibrate_ = create_service<std_srvs::srv::SetBool>("srv_calibrate_in", &VinsRepublisher::calibrateSrvCallback);
    srvs_calibrate_ = create_service<std_srvs::srv::SetBool>("~/calibrate_in", std::bind(&VinsRepublisher::calibrateSrvCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  is_initialized_ = true;
  timer_initialization_->cancel();
  //RCLCPP_INFO_ONCE(get_logger(), "[%s]: initialized", get_name());
}
//}

/* validateOdometry() //{ */

bool VinsRepublisher::validateOdometry(const nav_msgs::msg::Odometry &odometry) {

  // check position

  if (!std::isfinite(odometry.pose.pose.position.x)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.position.x'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.position.y)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.position.y'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.position.z)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.position.z'!!!");
    return false;
  }

  // check orientation

  if (!std::isfinite(odometry.pose.pose.orientation.x)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.orientation.x'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.orientation.y)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.orientation.y'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.orientation.z)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.orientation.z'!!!");
    return false;
  }

  if (!std::isfinite(odometry.pose.pose.orientation.w)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.pose.pose.orientation.w'!!!");
    return false;
  }

  // check if the quaternion is sound

  if (fabs(Eigen::Vector4d(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z,
                           odometry.pose.pose.orientation.w)
               .norm() -
           1.0) > 1e-2) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: orientation is not sound!!!");
    return false;
  }

  // check velocity

  if (!std::isfinite(odometry.twist.twist.linear.x)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.twist.twist.linear.x'!!!");
    return false;
  }

  if (!std::isfinite(odometry.twist.twist.linear.y)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.twist.twist.linear.y'!!!");
    return false;
  }

  if (!std::isfinite(odometry.twist.twist.linear.z)) {
    RATE_LIMITED_ERROR(1000, "[ControlManager]: NaN detected in variable 'odometry.twist.twist.linear.z'!!!");
    return false;
  }

  return true;
}

//}

/* odometryCallback() //{ */

void VinsRepublisher::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {

  if (!is_initialized_) {
    return;
  }

  if (!validateOdometry(*odom)) {
    RCLCPP_ERROR(get_logger(), "[VinsRepublisher]: input odometry is not numerically valid");
    return;
  }

  //RCLCPP_DEBUG(get_logger(), "[VinsRepublisher]: %d ", odom->header.seq);
  RCLCPP_DEBUG(get_logger(), "[VinsRepublisher]: %d ", odom->header.stamp.sec);
  RCLCPP_DEBUG(get_logger(), "[VinsRepublisher]: %d ", odom->header.stamp.nanosec);

  RCLCPP_DEBUG(get_logger(), "[now]: %f", get_clock()->now().seconds());
  RCLCPP_DEBUG(get_logger(), "[last published]: %f", publisher_odom_last_published_.seconds());
  RCLCPP_DEBUG(get_logger(), "[fabs diff]: %f", fabs((get_clock()->now() - publisher_odom_last_published_).seconds()));
  RCLCPP_DEBUG(get_logger(), "[diff]: %f", (get_clock()->now() - publisher_odom_last_published_).seconds());
  RCLCPP_DEBUG(get_logger(), "[rate_limiter]: %f", 1.0 / _rate_limiter_rate_);
  if (_rate_limiter_enabled_ && fabs((get_clock()->now() - publisher_odom_last_published_).seconds()) < (1.0 / (_rate_limiter_rate_))) {
    RCLCPP_DEBUG(get_logger(), "[%s]: skipping over", get_name());
    return;
  }

  nav_msgs::msg::Odometry odom_transformed;
  odom_transformed.header = odom->header;
  odom_transformed.header.frame_id = _mrs_vins_world_frame_;
  odom_transformed.child_frame_id = _fcu_frame_;

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
    RATE_LIMITED_WARNING(1000, "could not find transform from ", _fcu_frame_.c_str(), " to ", _vins_fcu_frame_.c_str());
    return;
  }

  /* transform pose with covariance */ /*//{*/
  geometry_msgs::msg::Pose pose_transformed;

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

  // save initial pose to subtract it from all messages to compensate initialized orientation ambiguity
  geometry_msgs::msg::TransformStamped tf_msg;
  if (_init_in_zero_) {
    if (!got_init_pose_) {
      init_hdg_ = mrs_lib::AttitudeConverter(pose_transformed.orientation).getHeading();
      RCLCPP_INFO(get_logger(), "[VinsRepublisher]: init hdg: %.2f", init_hdg_);
      got_init_pose_ = true;
    }

    // T^MRS_GLOBAL
    tf_msg.header.stamp            = odom->header.stamp;
    tf_msg.header.frame_id         = odom_transformed.header.frame_id;
    tf_msg.child_frame_id          = odom->header.frame_id;
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation      = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(-init_hdg_);

    // obtain T^MRS_FCU
    tf2::doTransform(pose_transformed, pose_transformed, tf_msg);

    // transform covariance
    // pose covariance is in world frame coordinates (in GLOBAL frame)
    // TODO is the orientation covariance also in world frame?

    // obtain transformed covariance as R^MRS_GLOBAL * covariance * (R^MRS_GLOBAL)^T where covariance is each 3x3 block of the 6x6 covariance matrix
    /* tf2::Stamped<tf2::Transform> tf; */
    tf2::Transform tf;
    tf2::fromMsg(tf_msg.transform, tf);
    odom_transformed.pose.covariance = transformCovariance(odom->pose.covariance, tf);
  } else {
    odom_transformed.pose.covariance = odom->pose.covariance;
  }

  // publish the initial offset to TF - T^GLOBAL_MRS
  geometry_msgs::msg::TransformStamped tf_msg_inv;
  tf_msg_inv.header.stamp            = odom->header.stamp;
  tf_msg_inv.header.frame_id         = odom->header.frame_id;
  tf_msg_inv.child_frame_id          = odom_transformed.header.frame_id;
  tf_msg_inv.transform.translation.x = 0;
  tf_msg_inv.transform.translation.y = 0;
  tf_msg_inv.transform.translation.z = 0;
  tf_msg_inv.transform.rotation      = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(init_hdg_);

  // try {
  //   broadcaster_->sendTransform(tf_msg_inv);
  // }
  // catch (...) {
  //   RCLCPP_ERROR(get_logger(), "[VinsRepublisher]: Exception caught during publishing TF: %s - %s.", tf_msg_inv.child_frame_id.c_str(), tf_msg_inv.header.frame_id.c_str());
  // }

  try {
    broadcaster_->sendTransform(tf_msg_inv);
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(get_logger(),
      "[VinsRepublisher]: TF Transform Exception during publishing TF: %s -> %s. "
      "Error: %s. Transform: [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f, %.3f], stamp: %f", 
      tf_msg_inv.header.frame_id.c_str(), 
      tf_msg_inv.child_frame_id.c_str(),
      ex.what(),
      tf_msg_inv.transform.translation.x,
      tf_msg_inv.transform.translation.y, 
      tf_msg_inv.transform.translation.z,
      tf_msg_inv.transform.rotation.x,
      tf_msg_inv.transform.rotation.y,
      tf_msg_inv.transform.rotation.z,
      tf_msg_inv.transform.rotation.w,
      tf_msg_inv.header.stamp.sec + tf_msg_inv.header.stamp.nanosec * 1e-9);
  }
  catch (const std::runtime_error& ex) {
    RCLCPP_ERROR(get_logger(),
      "[VinsRepublisher]: Runtime error during publishing TF: %s -> %s. "
      "Error: %s",
      tf_msg_inv.header.frame_id.c_str(),
      tf_msg_inv.child_frame_id.c_str(),
      ex.what());
  }
  catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(),
      "[VinsRepublisher]: Standard exception during publishing TF: %s -> %s. "
      "Error: %s. Transform data: pos[%.3f,%.3f,%.3f] rot[%.3f,%.3f,%.3f,%.3f]",
      tf_msg_inv.header.frame_id.c_str(),
      tf_msg_inv.child_frame_id.c_str(),
      ex.what(),
      tf_msg_inv.transform.translation.x,
      tf_msg_inv.transform.translation.y,
      tf_msg_inv.transform.translation.z,
      tf_msg_inv.transform.rotation.x,
      tf_msg_inv.transform.rotation.y,
      tf_msg_inv.transform.rotation.z,
      tf_msg_inv.transform.rotation.w);
  }
  catch (...) {
    RCLCPP_ERROR(get_logger(),
      "[VinsRepublisher]: Unknown exception during publishing TF: %s -> %s. "
      "Transform: pos[%.3f,%.3f,%.3f] rot[%.3f,%.3f,%.3f,%.3f] stamp: %f. "
      "Broadcaster state: %s",
      tf_msg_inv.header.frame_id.c_str(),
      tf_msg_inv.child_frame_id.c_str(),
      tf_msg_inv.transform.translation.x,
      tf_msg_inv.transform.translation.y,
      tf_msg_inv.transform.translation.z,
      tf_msg_inv.transform.rotation.x,
      tf_msg_inv.transform.rotation.y,
      tf_msg_inv.transform.rotation.z,
      tf_msg_inv.transform.rotation.w,
      tf_msg_inv.header.stamp.sec + tf_msg_inv.header.stamp.nanosec * 1e-9,
      broadcaster_ ? "valid" : "null");
  }

  odom_transformed.pose.pose = pose_transformed;
  /*//}*/

  /* transform velocity - linear and angular */ /*//{*/
  geometry_msgs::msg::Vector3 linear_velocity  = odom->twist.twist.linear;
  geometry_msgs::msg::Vector3 angular_velocity = odom->twist.twist.angular;
  if (_velocity_in_body_frame_) {
    // if in body frame - rotate from IMU frame to FCU frame
    Eigen::Vector3d v2;
    v2 << linear_velocity.x, linear_velocity.y, linear_velocity.z;
    v2                = R_IMU_FCU.transpose() * v2;
    linear_velocity.x = v2(0);
    linear_velocity.y = v2(1);
    linear_velocity.z = v2(2);

    Eigen::Vector3d v3;
    v3 << angular_velocity.x, angular_velocity.y, angular_velocity.z;
    v3                 = R_IMU_FCU.transpose() * v3;
    angular_velocity.x = v3(0);
    angular_velocity.y = v3(1);
    angular_velocity.z = v3(2);

  } else {
    /* if (_init_in_zero_) { */
    // if in global frame, apply initial TF offset
    /* tf2::doTransform(linear_velocity, linear_velocity, tf_msg); */
    /* tf2::doTransform(angular_velocity, angular_velocity, tf_msg); */
    /* } */

    // if in global frame - rotate from GLOBAL frame to FCU frame
    // R^GLOBAL_FCU = R^GLOBAL_IMU * R^IMU_FCU
    Eigen::Matrix3d R_GLOBAL_FCU = mrs_lib::AttitudeConverter(R_GLOBAL_IMU * R_IMU_FCU);
    Eigen::Vector3d v2;
    v2 << linear_velocity.x, linear_velocity.y, linear_velocity.z;
    v2                = R_GLOBAL_FCU.transpose() * v2;
    linear_velocity.x = v2(0);
    linear_velocity.y = v2(1);
    linear_velocity.z = v2(2);

    Eigen::Vector3d v3;
    v3 << angular_velocity.x, angular_velocity.y, angular_velocity.z;
    v3                 = R_GLOBAL_FCU.transpose() * v3;
    angular_velocity.x = v3(0);
    angular_velocity.y = v3(1);
    angular_velocity.z = v3(2);
  }

  odom_transformed.twist.twist.linear  = linear_velocity;
  odom_transformed.twist.twist.angular = angular_velocity;

  /*//}*/

  // validate
  if (!validateOdometry(odom_transformed)) {
    RCLCPP_ERROR(get_logger(), "[VinsRepublisher]: transformed odometry is not numerically valid");
    return;
  }

  /*//{ compensate initial tilt */
  if (compensate_initial_tilt_) {
    if (is_calibrated_) {

      // First subtract the initial position in the initialization frame
      odom_transformed.pose.pose.position.x -= odom_init_.pose.pose.position.x;
      odom_transformed.pose.pose.position.y -= odom_init_.pose.pose.position.y;
      odom_transformed.pose.pose.position.z -= odom_init_.pose.pose.position.z;

      // Rotate position to the calibrated frame
      Eigen::Vector3d pos;
      Eigen::Matrix3d init_rot = mrs_lib::AttitudeConverter(odom_init_.pose.pose.orientation);
      pos << odom_transformed.pose.pose.position.x, odom_transformed.pose.pose.position.y, odom_transformed.pose.pose.position.z;
      pos = init_rot.inverse() * pos;

      // Unrotate orientation
      const Eigen::Quaterniond q_init_rot   = mrs_lib::AttitudeConverter(init_rot.inverse());
      const Eigen::Quaterniond q_curr_rot   = mrs_lib::AttitudeConverter(odom_transformed.pose.pose.orientation);
      const Eigen::Quaterniond q_calibrated = q_init_rot * q_curr_rot;

      odom_transformed.pose.pose.orientation = mrs_lib::AttitudeConverter(q_calibrated);

      auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(odom_transformed.pose.pose.orientation).getExtrinsicRPY();
      //ROS_INFO_ONCE("[VinsRepublisher]: odom_transformed: t: (%.2f, %.2f, %.2f) [m] rpy: (%.2f, %.2f, %.2f) [deg]",
      //                  odom_transformed.pose.pose.position.x, odom_transformed.pose.pose.position.y, odom_transformed.pose.pose.position.z, roll * 180 / 3.14,
      //                  pitch * 180 / 3.14, yaw * 180 / 3.14);
    } else {
      mrs_lib::set_mutexed(mtx_odom_init_, odom_transformed, odom_init_);
      auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(odom_init_.pose.pose.orientation).getExtrinsicRPY();
      RATE_LIMITED_ERROR(1000, "init_odom: t: (%.2f, %.2f, %.2f) [m] rpy: (%.2f, %.2f, %.2f) [deg], waiting for calibration service call",
                        odom_init_.pose.pose.position.x, odom_init_.pose.pose.position.y, odom_init_.pose.pose.position.z, roll * 180 / 3.14,
                        pitch * 180 / 3.14, yaw * 180 / 3.14);
      has_valid_odom_ = true;
      return;
    }
  }
  /*//}*/

  // publish
  try {
    publisher_odom_->publish(odom_transformed);
    //RATE_LIMITED_LOG(1000, "Publishing");
    publisher_odom_last_published_ = get_clock()->now();
  }
  catch (...) {
    RCLCPP_ERROR(get_logger(), "exception caught during publishing topic '%s'", publisher_odom_->get_topic_name());
  }

  try {
    std::stringstream ss_pos;
    ss_pos << std::fixed << std::setprecision(2) << "VINS: X: " << odom_transformed.pose.pose.position.x << " Y: " << odom_transformed.pose.pose.position.y << " Z: " << odom_transformed.pose.pose.position.z;
    std_msgs::msg::String string_pos;
    string_pos.data = ss_pos.str();
    publisher_status_->publish(string_pos);
    //RATE_LIMITED_LOG(1000, "Publishing");
  }
  catch (...) {
    RCLCPP_ERROR(get_logger(), "exception caught during publishing topic '%s'", publisher_status_->get_topic_name());
  }

}

//}

/* transformCovariance() */ /*//{*/
// taken from https://github.com/ros/geometry2/blob/noetic-devel/tf2_geometry_msgs/include/tf2_geometry_msgs/tf2_geometry_msgs.h
// copied here, so that it's clear what it does right away
geometry_msgs::msg::PoseWithCovariance::_covariance_type VinsRepublisher::transformCovariance(const geometry_msgs::msg::PoseWithCovariance::_covariance_type &cov_in,
                                                                                         const tf2::Transform &                                     transform) {
  /**
   * To transform a covariance matrix:
   *
   * [R 0] COVARIANCE [R' 0 ]
   * [0 R]            [0  R']
   *
   * Where:
   * 	R is the rotation matrix (3x3).
   * 	R' is the transpose of the rotation matrix.
   * 	COVARIANCE is the 6x6 covariance matrix to be transformed.
   */

  // get rotation matrix transpose
  const tf2::Matrix3x3 R_transpose = transform.getBasis().transpose();

  // convert the covariance matrix into four 3x3 blocks
  const tf2::Matrix3x3 cov_11(cov_in[0], cov_in[1], cov_in[2], cov_in[6], cov_in[7], cov_in[8], cov_in[12], cov_in[13], cov_in[14]);
  const tf2::Matrix3x3 cov_12(cov_in[3], cov_in[4], cov_in[5], cov_in[9], cov_in[10], cov_in[11], cov_in[15], cov_in[16], cov_in[17]);
  const tf2::Matrix3x3 cov_21(cov_in[18], cov_in[19], cov_in[20], cov_in[24], cov_in[25], cov_in[26], cov_in[30], cov_in[31], cov_in[32]);
  const tf2::Matrix3x3 cov_22(cov_in[21], cov_in[22], cov_in[23], cov_in[27], cov_in[28], cov_in[29], cov_in[33], cov_in[34], cov_in[35]);

  // perform blockwise matrix multiplication
  const tf2::Matrix3x3 result_11 = transform.getBasis() * cov_11 * R_transpose;
  const tf2::Matrix3x3 result_12 = transform.getBasis() * cov_12 * R_transpose;
  const tf2::Matrix3x3 result_21 = transform.getBasis() * cov_21 * R_transpose;
  const tf2::Matrix3x3 result_22 = transform.getBasis() * cov_22 * R_transpose;

  // form the output
  geometry_msgs::msg::PoseWithCovariance::_covariance_type output;
  output[0]  = result_11[0][0];
  output[1]  = result_11[0][1];
  output[2]  = result_11[0][2];
  output[6]  = result_11[1][0];
  output[7]  = result_11[1][1];
  output[8]  = result_11[1][2];
  output[12] = result_11[2][0];
  output[13] = result_11[2][1];
  output[14] = result_11[2][2];

  output[3]  = result_12[0][0];
  output[4]  = result_12[0][1];
  output[5]  = result_12[0][2];
  output[9]  = result_12[1][0];
  output[10] = result_12[1][1];
  output[11] = result_12[1][2];
  output[15] = result_12[2][0];
  output[16] = result_12[2][1];
  output[17] = result_12[2][2];

  output[18] = result_21[0][0];
  output[19] = result_21[0][1];
  output[20] = result_21[0][2];
  output[24] = result_21[1][0];
  output[25] = result_21[1][1];
  output[26] = result_21[1][2];
  output[30] = result_21[2][0];
  output[31] = result_21[2][1];
  output[32] = result_21[2][2];

  output[21] = result_22[0][0];
  output[22] = result_22[0][1];
  output[23] = result_22[0][2];
  output[27] = result_22[1][0];
  output[28] = result_22[1][1];
  output[29] = result_22[1][2];
  output[33] = result_22[2][0];
  output[34] = result_22[2][1];
  output[35] = result_22[2][2];

  return output;
}
/*//}*/

/*//{ calibrateSrvCallback() */
bool VinsRepublisher::calibrateSrvCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {

  if (!has_valid_odom_) {
    RCLCPP_ERROR(get_logger(), "service for calibration called before obtaining valid odom.");
    return false;
  }

  RCLCPP_INFO(get_logger(), "calibrating level horizon.");
  auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(odom_init_.pose.pose.orientation).getExtrinsicRPY();
  //RATE_LIMITED_LOG(1000, "calibrated initial pose as: t: (%.2f, %.2f, %.2f) [m] rpy: (%.2f, %.f, %.2f) [deg]",
  //                  odom_init_.pose.pose.position.x, odom_init_.pose.pose.position.y, odom_init_.pose.pose.position.z, roll * 180 / 3.14, pitch * 180 / 3.14,
  //                  yaw * 180 / 3.14);

  res->success = true;
  res->message = "calibrated";

  is_calibrated_ = true;

  return true;
}
/*//}*/

/*//{ Exp() */
Eigen::Matrix3d VinsRepublisher::Exp(const Eigen::Vector3d &ang) {
  const double ang_norm = ang.norm();

  if (ang_norm < 0.0000001) {
    return Eigen::Matrix3d::Identity();
  } else {
    const Eigen::Vector3d rot_axis = ang / ang_norm;
    const Eigen::Matrix3d m_skew   = skewSymmetricMatrix(rot_axis);
    return Eigen::Matrix3d::Identity() + std::sin(ang_norm) * m_skew + (1.0 - std::cos(ang_norm)) * m_skew * m_skew;
  }
}
/*//}*/

/*//{ skewSymmetricMatrix() */
Eigen::Matrix3d VinsRepublisher::skewSymmetricMatrix(const Eigen::Vector3d &vec) {
  Eigen::Matrix3d m_skew;
  m_skew << 0.0, vec(2), -vec(1), -vec(2), 0.0, vec(0), vec(1), -vec(0), 0.0;
  return m_skew;
}
/*//}*/

}  // namespace vins_republisher
/* every nodelet must export its class as nodelet plugin */

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vins_republisher::VinsRepublisher)
