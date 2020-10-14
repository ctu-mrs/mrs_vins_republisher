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
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>

//}

ros::Publisher  publisher_odom_;
ros::Time publisher_odom_last_published_;
bool _rate_limiter_enabled_ = false;
double _rate_limiter_rate_;

ros::Subscriber subscriber_vins_;

std::string _uav_name_;
bool        is_initialized_ = false;

mrs_lib::Transformer transformer_;

std::string _camera_frame_;
std::string _fcu_frame_;
std::string _mrs_vins_world_frame_;
std::string _vins_fcu_frame_;

/* odometryCallback() //{ */

void odometryCallback(const nav_msgs::OdometryConstPtr &odom) {

  if (!is_initialized_) {
    return;
  }

  if (_rate_limiter_enabled_ && fabs((ros::Time::now() - publisher_odom_last_published_).toSec()) < (1.0/_rate_limiter_rate_)) {
    ROS_INFO("[%s]: skipping over", ros::this_node::getName().c_str());
    return; 
  }

  ROS_INFO("[%s]: publishing", ros::this_node::getName().c_str());

  geometry_msgs::PoseStamped vins_pose;
  vins_pose.header = odom->header;
  vins_pose.pose   = mrs_lib::getPose(odom);

  geometry_msgs::Vector3Stamped vins_velocity;
  vins_velocity.header = odom->header;
  vins_velocity.vector = odom->twist.twist.linear;

  geometry_msgs::Vector3Stamped vins_ang_velocity;
  vins_ang_velocity.header = odom->header;
  vins_ang_velocity.vector = odom->twist.twist.angular;

  mrs_lib::TransformStamped tf;

  /* get the transform from mrs_vins_world to vins_world //{ */
  
  {
    auto res = transformer_.getTransform(_mrs_vins_world_frame_, odom->header.frame_id, odom->header.stamp);
  
    if (!res) {
      ROS_WARN_THROTTLE(1.0, "[%s]: could not find transform from '%s' to '%s' at time '%f'", ros::this_node::getName().c_str(), _mrs_vins_world_frame_.c_str(),
                        odom->header.frame_id.c_str(), odom->header.stamp.toSec());
      return;
    }
  
    tf = res.value();
  }
  
  //}

  /* transform the vins pose to mrs_world_frame //{ */
  
  geometry_msgs::PoseStamped vins_pose_mrs_world;
  
  {
    auto res = transformer_.transform(tf, vins_pose);
  
    if (!res) {
      ROS_WARN_THROTTLE(1.0, "[%s]: could not transform vins pose to '%s'", ros::this_node::getName().c_str(), _mrs_vins_world_frame_.c_str());
      return;
    }
  
    vins_pose_mrs_world = res.value();
  }

  {
    auto res = transformer_.getTransform(_fcu_frame_, _vins_fcu_frame_, odom->header.stamp);
  
    if (!res) {
      ROS_WARN_THROTTLE(1.0, "[%s]: could not find transform from '%s' to '%s'", ros::this_node::getName().c_str(), _fcu_frame_.c_str(), _vins_fcu_frame_.c_str());
      return;
    }

    Eigen::Matrix3d rotation = mrs_lib::AttitudeConverter(res.value().getTransform().transform.rotation);

    /* rotation << 0, 0, 1, */
    /*             -1, 0, 0, */
    /*             0, -1, 0; */

    Eigen::Matrix3d original_orientation = mrs_lib::AttitudeConverter(vins_pose_mrs_world.pose.orientation);
    vins_pose_mrs_world.pose.orientation = mrs_lib::AttitudeConverter(original_orientation * rotation);
  }
  
  //}

  geometry_msgs::Vector3Stamped vins_velocity_mrs_world;

  /* transform the vins velocity to mrs_world_frame //{ */
  
  {
    auto res = transformer_.transform(tf, vins_velocity);
  
    if (!res) {
      ROS_WARN_THROTTLE(1.0, "[%s]: could not transform vins velocity to '%s'", ros::this_node::getName().c_str(), _mrs_vins_world_frame_.c_str());
      return;
    }
  
    vins_velocity_mrs_world = res.value();
  }
  
  //}

  geometry_msgs::Vector3Stamped vins_ang_velocity_mrs_world;

  /* transform the vins angular velocity to mrs_world_frame //{ */
  
  {
    auto res = transformer_.transform(tf, vins_ang_velocity);
  
    if (!res) {
      ROS_WARN_THROTTLE(1.0, "[%s]: could not transform vins angular velocity to '%s'", ros::this_node::getName().c_str(), _mrs_vins_world_frame_.c_str());
      return;
    }
  
    vins_ang_velocity_mrs_world = res.value();
  }
  
  //}

  geometry_msgs::Vector3 camera_to_fcu_translation;

  /* find transform from camera frame to fcu frame //{ */
  
  {
    auto res = transformer_.getTransform(_camera_frame_, _fcu_frame_, odom->header.stamp);
  
    if (!res) {
  
      ROS_WARN_THROTTLE(1.0, "[%s]: could not find transform from '%s' to '%s'", ros::this_node::getName().c_str(), _camera_frame_.c_str(),
                        _fcu_frame_.c_str());
      return;
    }
  
    camera_to_fcu_translation = res.value().getTransform().transform.translation;
  }
  
  //}

  // fill the new transformed odom message

  nav_msgs::Odometry odom_trans;

  odom_trans.header.stamp    = odom->header.stamp;
  odom_trans.header.frame_id = _mrs_vins_world_frame_;

  odom_trans.pose.pose           = vins_pose_mrs_world.pose;
  odom_trans.twist.twist.linear  = vins_velocity_mrs_world.vector;
  odom_trans.twist.twist.angular = vins_ang_velocity_mrs_world.vector;

  try {
    publisher_odom_.publish(odom_trans);

    publisher_odom_last_published_ = ros::Time::now();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_odom_.getTopic().c_str());
  }
}

//}

/* main() //{ */

int main(int argc, char **argv) {

  const std::string node_name("VinsRepublisher");

  ros::init(argc, argv, node_name);
  ros::NodeHandle nh("~");

  ros::Time::waitForValid();

  publisher_odom_last_published_ = ros::Time(0);

  ROS_INFO("[%s]: loading parameters using ParamLoader", node_name.c_str());

  mrs_lib::ParamLoader pl(nh, node_name);

  pl.loadParam("uav_name", _uav_name_);

  pl.loadParam("rate_limiter/enabled", _rate_limiter_enabled_);
  pl.loadParam("rate_limiter/max_rate", _rate_limiter_rate_);

  if (_rate_limiter_rate_ <= 1e-3) {
    ROS_ERROR("[%s]: the rate limit has to be > 0", ros::this_node::getName().c_str());
    ros::shutdown(); 
  }

  pl.loadParam("fcu_frame", _fcu_frame_);
  pl.loadParam("camera_frame", _camera_frame_);
  pl.loadParam("mrs_vins_world_frame", _mrs_vins_world_frame_);
  pl.loadParam("vins_fcu_frame", _vins_fcu_frame_);

  if (!pl.loadedSuccessfully()) {
    ROS_ERROR("[%s]: parameter loading failure", node_name.c_str());
    ros::shutdown();
  }

  transformer_ = mrs_lib::Transformer("VinsRepublisher", "");

  /* nav_msgs::Odometry_<std::allocator<void>> odom_main; */

  // | ----------------------- subscirbers ---------------------- |

  subscriber_vins_ = nh.subscribe("vins_odom_in", 1, odometryCallback);

  // | ----------------------- publishers ----------------------- |

  publisher_odom_ = nh.advertise<nav_msgs::Odometry>("vins_odom_out", 1);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ros::spin();

  return 0;
};

//}
