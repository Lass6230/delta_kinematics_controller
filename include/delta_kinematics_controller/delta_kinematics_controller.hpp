#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// forward-declare kinematics_interface base class to avoid hard dependency in header
namespace kinematics_interface { class KinematicsInterface; }
#include <Eigen/Dense>

namespace delta_kinematics_controller
{

class DeltaKinematicsController : public controller_interface::ControllerInterface
{
public:
  DeltaKinematicsController();
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // parameters
  std::vector<std::string> joint_names_;
  std::vector<std::string> lower_joint_names_;
  std::string kinematics_plugin_name_ = "kinematics_interface_delta/DeltaKinematicsPlugin";

  // plugin loader and instance
  std::unique_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> kinematics_loader_;
  std::shared_ptr<kinematics_interface::KinematicsInterface> kinematics_plugin_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  // TF broadcaster for base->ee
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // TF publish rate (Hz). 0 disables TF broadcasting.
  double ee_tf_rate_ = 50.0;
  rclcpp::Time last_ee_tf_time_ = rclcpp::Time(0);

  // Configurable frame names
  std::string base_link_ = "base";
  std::string ee_link_ = "end_effector";
};

} // namespace
