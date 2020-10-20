
/*includes//{*/
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

#include <nodelet/nodelet.h>/*//}*/

/* class VinsRepublisher//{*/
namespace vins_republisher
{

class VinsRepublisher : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  /* flags */
  bool        is_initialized_ = false;
  bool       _rate_limiter_enabled_ = false;

  /* ros parameters */
  std::string _uav_name_;
  std::string _camera_frame_;
  std::string _fcu_frame_;
  std::string _mrs_vins_world_frame_;
  std::string _vins_fcu_frame_;

  // | ------------------------ callbacks ----------------------- |
  ros::Subscriber subscriber_vins_;
  void odometryCallback(const nav_msgs::OdometryConstPtr &odom);

  ros::Publisher publisher_odom_;
  ros::Time      publisher_odom_last_published_;
  double         _rate_limiter_rate_;

  /* transformation handler */
  mrs_lib::Transformer transformer_;

};
}  // namespace vins_republisher
   /*//}*/
