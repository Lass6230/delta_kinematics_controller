#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <kinematics_interface/kinematics_interface/kinematics_interface.hpp>
#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class DeltaControllerNode : public rclcpp::Node
{
public:
  DeltaControllerNode(): Node("delta_controller_node")
  {
    this->declare_parameter<std::vector<std::string>>("joints", {"joint1","joint2","joint3"});
    this->declare_parameter<std::string>("kinematics_plugin", "kinematics_interface_delta/DeltaKinematicsPlugin");
    this->get_parameter("joints", joint_names_);
    this->get_parameter("kinematics_plugin", kinematics_plugin_name_);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg){
        std::lock_guard<std::mutex> lk(mutex_);
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
          auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
          if (it != msg->name.end())
          {
            size_t idx = std::distance(msg->name.begin(), it);
            if (idx < msg->position.size()) current_positions_[i] = msg->position[idx];
          }
        }
      });

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("cmd_twist", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg){
        std::lock_guard<std::mutex> lk(mutex_);
        last_twist_ = *msg;
      });

    joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);

    // plugin loader
    try
    {
      loader_ = std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>("kinematics_interface", "kinematics_interface::KinematicsInterface");
      kinematics_ = loader_->createUniqueInstance(kinematics_plugin_name_);
      if (kinematics_)
      {
        // initialize plugin (no NodeParametersInterface available here; pass nullptr)
        kinematics_->initialize("delta_controller", nullptr, "ee");
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load kinematics plugin: %s", e.what());
    }

    current_positions_.assign(joint_names_.size(), 0.0);

    timer_ = this->create_wall_timer(10ms, std::bind(&DeltaControllerNode::update, this));
  }

private:
  void update()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    // compute delta = twist * dt
    double dt = 0.01; // 10ms
    Eigen::Matrix<double, 6, 1> cart_delta = Eigen::Matrix<double, 6, 1>::Zero();
    cart_delta(0) = last_twist_.twist.linear.x * dt;
    cart_delta(1) = last_twist_.twist.linear.y * dt;
    cart_delta(2) = last_twist_.twist.linear.z * dt;
    cart_delta(3) = last_twist_.twist.angular.x * dt;
    cart_delta(4) = last_twist_.twist.angular.y * dt;
    cart_delta(5) = last_twist_.twist.angular.z * dt;

    Eigen::VectorXd joint_angles(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) joint_angles(i) = current_positions_[i];

    Eigen::VectorXd joint_delta(joint_names_.size()); joint_delta.setZero();
    if (kinematics_)
    {
      bool ok = kinematics_->convert_cartesian_deltas_to_joint_deltas(joint_angles, cart_delta, "", joint_delta);
      if (!ok) return; // skip if conversion fails
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = joint_names_;
    js.position.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      current_positions_[i] += joint_delta(i);
      js.position[i] = current_positions_[i];
    }
    joint_cmd_pub_->publish(js);
  }

  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  std::string kinematics_plugin_name_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::TwistStamped last_twist_;
  std::mutex mutex_;

  std::unique_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> loader_;
  pluginlib::UniquePtr<kinematics_interface::KinematicsInterface> kinematics_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeltaControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
