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
#include <tf2_ros/static_transform_broadcaster.h>

ros::Publisher  odom_pub;
ros::Publisher  rpy_pub;
ros::Subscriber sub;

geometry_msgs::TransformStamped transformStamped;
tf2_ros::Buffer                 tfBuffer;
double                          is_odom_main;
tf2::Quaternion                 q_new, q_rot;
std::string                     header_frame;

void odometryCallback(const nav_msgs::OdometryConstPtr &odom) {
  nav_msgs::Odometry                       odom_trans;
  geometry_msgs::PoseWithCovarianceStamped pose_stamped;
  geometry_msgs::Vector3                   vect3;
  geometry_msgs::Vector3Stamped            rpy;
  tf2::Quaternion                          q;

  /* Transform the Odometry message */
  pose_stamped.header = odom->header;
  pose_stamped.pose   = odom->pose;
  tf2::doTransform(pose_stamped, pose_stamped, transformStamped);
  odom_trans.header          = odom->header;
  odom_trans.pose            = pose_stamped.pose;
  odom_trans.header.frame_id = pose_stamped.header.frame_id;

  /* only for vins-mono */
  odom_trans.pose.pose.orientation = odom->pose.pose.orientation;

  /* tf2::fromMsg(odom_trans.pose.pose.orientation, q); */
  /* q_rot = tf2::Quaternion(0.7071068, 0, 0, 0.7071068); */
  /* 30 deg */
  /* q_rot = tf2::Quaternion(0, -0.258819, 0, 0.9659258); */
  /* q_rot = tf2::Quaternion(0, -0.7071068, 0, 0.7071068); */
  /* q_rot = tf2::Quaternion(0, 0, 0, 1); */
  /* q_new = q_rot * q; */
  /* q_new.normalize(); */
  /* tf2::convert(q_new, odom_trans.pose.pose.orientation); */

  vect3 = odom->twist.twist.angular;
  tf2::doTransform(vect3, vect3, transformStamped);
  odom_trans.twist.twist.angular = vect3;

  vect3 = odom->twist.twist.linear;
  tf2::doTransform(vect3, vect3, transformStamped);
  odom_trans.twist.twist.linear = vect3;

  /* Create yaw/pitch/roll vector */
  rpy.header = odom_trans.header;
  tf2::fromMsg(pose_stamped.pose.pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

  /* ROS_INFO("publish"); */
  rpy_pub.publish(rpy);
  odom_pub.publish(odom_trans);
  /* ROS_INFO("after_publish"); */
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vins_republish_node");
  ros::NodeHandle node("~");

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  node.param<double>("is_odom_main", is_odom_main, 0);
  ROS_INFO("is_odom_main: %f", is_odom_main);
  node.param<std::string>("header_frame", header_frame, "vio_tf");
  ROS_INFO("header_frame: %s", header_frame.c_str());

  nav_msgs::Odometry_<std::allocator<void>> odom_main;
  if (is_odom_main) {
    odom_main = *(ros::topic::waitForMessage<nav_msgs::Odometry>("odometry/odom_main"));

    transformStamped.transform.translation.x = odom_main.pose.pose.position.x;
    transformStamped.transform.translation.y = odom_main.pose.pose.position.y;
    transformStamped.transform.translation.z = odom_main.pose.pose.position.z;
    transformStamped.transform.rotation.x    = odom_main.pose.pose.orientation.x;
    transformStamped.transform.rotation.y    = odom_main.pose.pose.orientation.y;
    transformStamped.transform.rotation.z    = odom_main.pose.pose.orientation.z;
    transformStamped.transform.rotation.w    = odom_main.pose.pose.orientation.w;
  } else {
    ROS_INFO("no odom main");
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;
    transformStamped.transform.rotation.x    = 0;
    transformStamped.transform.rotation.y    = 0;
    transformStamped.transform.rotation.z    = 0.707;
    transformStamped.transform.rotation.w    = 0.707;
  }

  nav_msgs::Odometry_<std::allocator<void>> odom_vins = *(ros::topic::waitForMessage<nav_msgs::Odometry>("vins_odom_in", node));

  transformStamped.header.stamp    = ros::Time::now();
  transformStamped.header.frame_id = header_frame;
  transformStamped.child_frame_id  = "world";

  sub      = node.subscribe("vins_odom_in", 1000, odometryCallback);
  odom_pub = node.advertise<nav_msgs::Odometry>("vins_odom_out", 1000);
  rpy_pub  = node.advertise<geometry_msgs::Vector3Stamped>("rpy_out", 1000);

  static_broadcaster.sendTransform(transformStamped);

  ros::spin();

  return 0;
};
