
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class PM01NavBridge : public rclcpp::Node
{
public:
  PM01NavBridge()
  : Node("pm01_nav_bridge"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    // Provisional Livox position relative to LINK_TORSO_YAW.
    // 0.30 m measured pelvis-to-Livox height
    // minus the URDF's 0.0809 m torso-joint height.
    mount_x_ = this->declare_parameter<double>("mount_x", 0.12438064);
    mount_y_ = this->declare_parameter<double>("mount_y", 0.0);
    mount_z_ = this->declare_parameter<double>("mount_z", 0.2128948);

    nav_odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 10);

    fastlio_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry",
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        std::bind(&PM01NavBridge::odom_callback, this, _1));

    // Navigation-frame rotation:
    // odom -> camera_init = -90 degrees around Y.
    //
    // This is the inverse of the previously tested
    // camera_init -> nav_test (+90 degrees around Y).
    tf2::Quaternion q_world;
    q_world.setRPY(0.0, -M_PI / 2.0, 0.0);
    q_world.normalize();

    t_odom_camera_init_.setIdentity();
    t_odom_camera_init_.setRotation(q_world);

    // Candidate Livox orientation relative to PM01 torso.
    //
    // Livox +X = up
    // Livox -Y = left
    // Livox +Z = forward
    //
    // Quaternion was previously verified in tf2_echo.
    tf2::Quaternion q_mount(
      0.7071067811865476,
      0.0,
      0.7071067811865476,
      0.0);

    q_mount.normalize();

    t_torso_body_.setIdentity();
    t_torso_body_.setOrigin(
      tf2::Vector3(mount_x_, mount_y_, mount_z_));
    t_torso_body_.setRotation(q_mount);

    RCLCPP_INFO(
      this->get_logger(),
      "PM01 nav bridge started. Mount offset: "
      "x=%.4f y=%.4f z=%.4f m",
      mount_x_, mount_y_, mount_z_);

    RCLCPP_WARN(
      this->get_logger(),
      "Pose/TF validation only. Velocity is not yet "
      "estimated; do not enable autonomous locomotion.");
  }

private:
  static tf2::Transform transform_from_pose(
    const geometry_msgs::msg::Pose & pose)
  {
    tf2::Transform transform;

    transform.setOrigin(tf2::Vector3(
      pose.position.x,
      pose.position.y,
      pose.position.z));

    transform.setRotation(tf2::Quaternion(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w));

    return transform;
  }

  static tf2::Transform transform_from_tf(
    const geometry_msgs::msg::Transform & msg)
  {
    tf2::Transform transform;

    transform.setOrigin(tf2::Vector3(
      msg.translation.x,
      msg.translation.y,
      msg.translation.z));

    transform.setRotation(tf2::Quaternion(
      msg.rotation.x,
      msg.rotation.y,
      msg.rotation.z,
      msg.rotation.w));

    return transform;
  }

  void odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id != "camera_init" ||
        msg->child_frame_id != "body")
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Unexpected FAST-LIO frame IDs: %s -> %s",
        msg->header.frame_id.c_str(),
        msg->child_frame_id.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped waist_tf;


    try
    {
      // Use the waist angle at the FAST-LIO pose timestamp.
      waist_tf = tf_buffer_.lookupTransform(
        "LINK_BASE",
        "LINK_TORSO_YAW",
        rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(0.05));
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for timestamp-matched waist TF: %s",
        ex.what());
      return;
    }

    // FAST-LIO's estimated pose: camera_init -> body.
    const tf2::Transform t_camera_body =
      transform_from_pose(msg->pose.pose);

    // Robot kinematics: LINK_BASE -> LINK_TORSO_YAW.
    const tf2::Transform t_base_torso =
      transform_from_tf(waist_tf.transform);

    // Robot kinematics: LINK_BASE -> body.
    //
    // Includes both moving waist yaw and fixed Livox mount.
    const tf2::Transform t_base_body =
      t_base_torso * t_torso_body_;

    // Convert FAST-LIO sensor pose into navigation pelvis pose.
    const tf2::Transform t_odom_base =
      t_odom_camera_init_ *
      t_camera_body *
      t_base_body.inverse();

    const tf2::Vector3 p = t_odom_base.getOrigin();
    const tf2::Quaternion q = t_odom_base.getRotation();

    // Publish dynamic navigation TF.
    geometry_msgs::msg::TransformStamped nav_tf;
    nav_tf.header.stamp = msg->header.stamp;
    nav_tf.header.frame_id = "odom";
    nav_tf.child_frame_id = "LINK_BASE";

    nav_tf.transform.translation.x = p.x();
    nav_tf.transform.translation.y = p.y();
    nav_tf.transform.translation.z = p.z();

    nav_tf.transform.rotation.x = q.x();
    nav_tf.transform.rotation.y = q.y();
    nav_tf.transform.rotation.z = q.z();
    nav_tf.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(nav_tf);

    // Publish pose-only provisional navigation odometry.
    nav_msgs::msg::Odometry nav_odom;

    nav_odom.header.stamp = msg->header.stamp;
    nav_odom.header.frame_id = "odom";
    nav_odom.child_frame_id = "LINK_BASE";

    nav_odom.pose.pose.position.x = p.x();
    nav_odom.pose.pose.position.y = p.y();
    nav_odom.pose.pose.position.z = p.z();

    nav_odom.pose.pose.orientation.x = q.x();
    nav_odom.pose.pose.orientation.y = q.y();
    nav_odom.pose.pose.orientation.z = q.z();
    nav_odom.pose.pose.orientation.w = q.w();

    // Provisional conservative pose uncertainty.
    // These are placeholders, NOT calibrated covariances.
    nav_odom.pose.covariance.fill(0.0);
    nav_odom.pose.covariance[0]  = 0.25;
    nav_odom.pose.covariance[7]  = 0.25;
    nav_odom.pose.covariance[14] = 0.25;
    nav_odom.pose.covariance[21] = 0.25;
    nav_odom.pose.covariance[28] = 0.25;
    nav_odom.pose.covariance[35] = 0.25;

    // FAST-LIO currently outputs all-zero twist.
    // Do not interpret zeros as measured robot velocity.
    // Assign very high velocity uncertainty.
    nav_odom.twist.covariance.fill(0.0);
    for (int index : {0, 7, 14, 21, 28, 35})
    {
      nav_odom.twist.covariance[index] = 1e6;
    }

    nav_odom_pub_->publish(nav_odom);
  }

  double mount_x_;
  double mount_y_;
  double mount_z_;

  tf2::Transform t_odom_camera_init_;
  tf2::Transform t_torso_body_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    fastlio_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    nav_odom_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PM01NavBridge>());
  rclcpp::shutdown();
  return 0;
}