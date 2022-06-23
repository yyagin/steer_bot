// ROS Message Headers
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

// ROS Related Headers
#include "ros/ros.h"
#include "ros/topic.h"

// TF2 Headers
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tf2_broadcaster");
  ros::NodeHandle nh;

  bool callback_flag{false};
  int publish_rate{50};
  std::string topic_name;

  nh.param<int>(ros::this_node::getName() + "/publish_rate", publish_rate, 50);
  nh.param<std::string>(ros::this_node::getName() + "/topic_name", topic_name,
                        "/odom");

  geometry_msgs::TransformStamped tf_msg;
  nav_msgs::Odometry odom_msg;
  tf2_ros::TransformBroadcaster broadcaster;

  auto sub = nh.subscribe<nav_msgs::Odometry>(
      topic_name, 100,
      [&odom_msg, &callback_flag](const nav_msgs::Odometry::ConstPtr &msg) {
        odom_msg = *msg;
        callback_flag = true;
      });

  auto timer =
      nh.createTimer(ros::Duration{1.0 / publish_rate},
                     [&odom_msg, &broadcaster, &tf_msg,
                      &callback_flag](const ros::TimerEvent & /*event*/) {
                       const auto &p = odom_msg.pose.pose.position;
                       const auto &q = odom_msg.pose.pose.orientation;
                       tf_msg.header.stamp = ros::Time::now();
                       tf_msg.header.frame_id = odom_msg.header.frame_id;
                       tf_msg.child_frame_id = odom_msg.child_frame_id;
                       tf_msg.transform.translation.x = p.x;
                       tf_msg.transform.translation.y = p.y;
                       tf_msg.transform.translation.z = p.z;
                       tf_msg.transform.rotation.x = q.x;
                       tf_msg.transform.rotation.y = q.y;
                       tf_msg.transform.rotation.z = q.z;
                       tf_msg.transform.rotation.w = q.w;
                       if (callback_flag) {
                         broadcaster.sendTransform(tf_msg);
                       }
                     });

  ros::spin();
  return EXIT_SUCCESS;
}
