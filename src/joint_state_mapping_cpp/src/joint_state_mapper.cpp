#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "interface_protocol/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateMapper : public rclcpp::Node
{
public:
  JointStateMapper()
  : Node("joint_state_mapper"),
    joint_names_{
      "J00_HIP_PITCH_L",
      "J01_HIP_ROLL_L",
      "J02_HIP_YAW_L",
      "J03_KNEE_PITCH_L",
      "J04_ANKLE_PITCH_L",
      "J05_ANKLE_ROLL_L",
      "J06_HIP_PITCH_R",
      "J07_HIP_ROLL_R",
      "J08_HIP_YAW_R",
      "J09_KNEE_PITCH_R",
      "J10_ANKLE_PITCH_R",
      "J11_ANKLE_ROLL_R",
      "J12_WAIST_YAW",
      "J13_SHOULDER_PITCH_L",
      "J14_SHOULDER_ROLL_L",
      "J15_SHOULDER_YAW_L",
      "J16_ELBOW_PITCH_L",
      "J17_ELBOW_YAW_L",
      "J18_SHOULDER_PITCH_R",
      "J19_SHOULDER_ROLL_R",
      "J20_SHOULDER_YAW_R",
      "J21_ELBOW_PITCH_R",
      "J22_ELBOW_YAW_R",
      "J23_HEAD_YAW"
    }
  {
    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(10));

    joint_state_sub_ =
      create_subscription<interface_protocol::msg::JointState>(
        "/hardware/joint_state",
        rclcpp::SensorDataQoS(),
        std::bind(
          &JointStateMapper::jointStateCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Mapping /hardware/joint_state to /joint_states");
  }

private:
  void jointStateCallback(
    const interface_protocol::msg::JointState::SharedPtr source)
  {
    if (source->position.size() != joint_names_.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Expected %zu joint positions, received %zu",
        joint_names_.size(),
        source->position.size());
      return;
    }

    sensor_msgs::msg::JointState output;

    output.header = source->header;
    output.name = joint_names_;
    output.position = source->position;

    if (source->velocity.size() == joint_names_.size()) {
      output.velocity = source->velocity;
    }

    if (source->torque.size() == joint_names_.size()) {
      output.effort = source->torque;
    }

    joint_state_pub_->publish(output);
  }

  const std::vector<std::string> joint_names_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_pub_;

  rclcpp::Subscription<interface_protocol::msg::JointState>::SharedPtr
    joint_state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateMapper>());
  rclcpp::shutdown();
  return 0;
}
