#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_odometry_transformer");
  ros::NodeHandle nh_private("~");

  tf2_ros::TransformBroadcaster tf_br_;

  // Settings
  double r, h, T;
  nh_private.param<double>("radius", r, 3.0);
  nh_private.param<double>("height", h, 10.0);
  nh_private.param<double>("period", T, 30.0);
  const double period = 2 * M_PI / T;
  ros::Rate loop_rate(10);

  geometry_msgs::TransformStamped tf;
  nav_msgs::Odometry odom;

  auto odom_pub =
      nh_private.advertise<nav_msgs::Odometry>("camera_odometry", 1);

  while (ros::ok()) {
    const auto t = ros::Time::now();
    const auto s = std::sin(period * tf.header.stamp.toSec());
    const auto c = std::cos(period * tf.header.stamp.toSec());
    const auto rs = r * s;
    const auto rc = r * c;

    // Simulated state.
    const Eigen::Vector3d p_W(rc, rs, h * s);
    const Eigen::Vector3d v_W = period * Eigen::Vector3d(-rs, rc, h * c);
    const auto xb = v_W.normalized();
    const auto yb = (-1.0 * p_W).normalized();
    const auto zb = xb.cross(yb);
    const Eigen::Matrix3d R_WC((Eigen::Matrix3d() << xb, yb, zb).finished());

    // Pose.
    const auto T_WC =
        Eigen::Translation3d(r * c, r * s, h * s) * Eigen::Quaterniond(R_WC);
    // Linear twist.
    const auto v_C = v_W.norm() * Eigen::Vector3d::UnitX();

    // Angular twist.
    const auto p_W_abs = p_W.norm();
    auto w_W = p_W.cross(v_W);
    w_W *= 1.0 / (p_W_abs * p_W_abs);

    // Broadcast.
    // tf2
    tf = tf2::eigenToTransform(T_WC);
    tf.header.stamp = t;
    tf.header.frame_id = "world";
    tf.child_frame_id = "camera";
    tf_br_.sendTransform(tf);

    // odometry
    odom.pose.pose = tf2::toMsg(T_WC);
    tf2::toMsg(v_C, odom.twist.twist.linear);
    tf2::toMsg(R_WC.inverse() * w_W, odom.twist.twist.angular);
    odom.header = tf.header;
    odom.child_frame_id = tf.child_frame_id;
    odom_pub.publish(odom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
