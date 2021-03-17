#include <VinsRepublisher.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>


namespace vins_republisher
{

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
  param_loader.loadParam("rate_limiter/enabled", _rate_limiter_enabled_);
  param_loader.loadParam("rate_limiter/max_rate", _rate_limiter_rate_);
  if (_rate_limiter_rate_ <= 1e-3) {
    ROS_ERROR("[%s]: the rate limit has to be > 0", ros::this_node::getName().c_str());
    ros::shutdown(); 
  }

  param_loader.loadParam("fcu_frame", _fcu_frame_);
  param_loader.loadParam("camera_frame", _camera_frame_);
  param_loader.loadParam("mrs_vins_world_frame", _mrs_vins_world_frame_);
  param_loader.loadParam("vins_fcu_frame", _vins_fcu_frame_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: parameter loading failure", node_name.c_str());
    ros::shutdown();
  }

  /* transformation handler */
  transformer_ = mrs_lib::Transformer("VinsRepublisher", "");

  // | ----------------------- subscribers ---------------------- |

  subscriber_vins_ = nh_.subscribe("vins_odom_in", 10, &VinsRepublisher::odometryCallback, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_odom_ = nh_.advertise<nav_msgs::Odometry>("vins_odom_out", 10);

  is_initialized_ = true;

  ROS_INFO_ONCE("[%s]: initialized",node_name.c_str());
}
//}

/* odometryCallback() //{ */

void VinsRepublisher::odometryCallback(const nav_msgs::OdometryConstPtr &odom) {
  ros::WallTime start_, end_;
  ROS_DEBUG("[]: ");
  if (!is_initialized_) {
    return;
  }
  ROS_DEBUG("[VinsRepublisher]: %d ", odom->header.seq);
  ROS_DEBUG("[VinsRepublisher]: %d ", odom->header.stamp.sec);
  ROS_DEBUG("[VinsRepublisher]: %d ", odom->header.stamp.nsec);
  start_ = ros::WallTime::now();

  ROS_DEBUG("[now]: %f",ros::Time::now().toSec());
  ROS_DEBUG("[last published]: %f",publisher_odom_last_published_.toSec());
  ROS_DEBUG("[fabs diff]: %f",fabs((ros::Time::now() - publisher_odom_last_published_).toSec()));
  ROS_DEBUG("[diff]: %f",(ros::Time::now() - publisher_odom_last_published_).toSec());
  ROS_DEBUG("[rate_limiter]: %f",1.0/_rate_limiter_rate_ );
  if (_rate_limiter_enabled_ && fabs((ros::Time::now() - publisher_odom_last_published_).toSec()) < (1.0/(_rate_limiter_rate_))) {
    ROS_DEBUG("[%s]: skipping over", ros::this_node::getName().c_str());
    return; 
  }

  ROS_DEBUG("[%s]: publishing", ros::this_node::getName().c_str());

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
  
  /* Vins-mono uses different coordinate system. The y-axis is front, x-axis is right, z-axis is up */
  /* This transformation rotates in yaw of 90 deg */
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
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_DEBUG_STREAM("Exectution time (ms): " << execution_time);
    publisher_odom_.publish(odom_trans);
    ROS_INFO_THROTTLE(1.0, "[%s]: Publishing", ros::this_node::getName().c_str());
    publisher_odom_last_published_ = ros::Time::now();
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", publisher_odom_.getTopic().c_str());
  }
}

//}

}
/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(vins_republisher::VinsRepublisher, nodelet::Nodelet);
