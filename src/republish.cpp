#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>

ros::Publisher  odom_pub;
ros::Publisher  rpy_pub;
ros::Subscriber sub;

geometry_msgs::TransformStamped             transformStamped;
tf2_ros::Buffer                             tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;

void odometryCallback(const nav_msgs::OdometryConstPtr &odom) {
  nav_msgs::Odometry                       odom_trans;
  geometry_msgs::PoseWithCovarianceStamped pose_stamped;
  geometry_msgs::Vector3                   vect3;
  geometry_msgs::Vector3Stamped            rpy;
  tf2::Quaternion                          q;

  try {
    transformStamped = tfBuffer.lookupTransform("local_origin", "world", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform local_origin to world: %s", ex.what());
  }

  /* Transform the Odometry message */
  pose_stamped.header = odom->header;
  pose_stamped.pose   = odom->pose;
  tf2::doTransform(pose_stamped, pose_stamped, transformStamped);
  odom_trans.pose   = pose_stamped.pose;
  odom_trans.header = pose_stamped.header;

  vect3 = odom->twist.twist.angular;
  tf2::doTransform(vect3, vect3, transformStamped);
  odom_trans.twist.twist.angular = vect3;

  vect3 = odom->twist.twist.linear;
  tf2::doTransform(vect3, vect3, transformStamped);
  odom_trans.twist.twist.linear = vect3;

  /* Create yaw/pitch/roll vector */
  rpy.header = pose_stamped.header;
  tf2::fromMsg(pose_stamped.pose.pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

  /* ROS_INFO("publish"); */
  rpy_pub.publish(rpy);
  odom_pub.publish(odom_trans);
  /* ROS_INFO("after_publish"); */
}

void postTransformation(const ros::TimerEvent &) {
  static tf2_ros::TransformBroadcaster br;
  transformStamped.header.stamp = ros::Time::now();
  br.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vins_republish_node");
  ros::NodeHandle node("~");

  nav_msgs::Odometry_<std::allocator<void>> odom_main = *(ros::topic::waitForMessage<nav_msgs::Odometry>("odom_main_in", node));
  nav_msgs::Odometry_<std::allocator<void>> odom_vins = *(ros::topic::waitForMessage<nav_msgs::Odometry>("vins_odom_in", node));

  transformStamped.header.stamp            = ros::Time::now();
  transformStamped.header.frame_id         = "local_origin";
  transformStamped.child_frame_id          = "world";
  transformStamped.transform.translation.x = odom_main.pose.pose.position.x;
  transformStamped.transform.translation.y = odom_main.pose.pose.position.y;
  transformStamped.transform.translation.z = odom_main.pose.pose.position.z;
  transformStamped.transform.rotation.x    = odom_main.pose.pose.orientation.x;
  transformStamped.transform.rotation.y    = odom_main.pose.pose.orientation.y;
  transformStamped.transform.rotation.z    = odom_main.pose.pose.orientation.z;
  transformStamped.transform.rotation.w    = odom_main.pose.pose.orientation.w;

  ros::Timer timer = node.createTimer(ros::Duration(0.001), postTransformation);
  sub              = node.subscribe("vins_odom_in", 1000, odometryCallback);
  odom_pub         = node.advertise<nav_msgs::Odometry>("vins_odom_out", 1000);
  rpy_pub          = node.advertise<geometry_msgs::Vector3Stamped>("rpy_out", 1000);

  tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);

  ros::spin();

  return 0;
};
