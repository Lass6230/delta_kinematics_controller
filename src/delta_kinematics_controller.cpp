#include "delta_kinematics_controller/delta_kinematics_controller.hpp"

#include <controller_interface/controller_interface.hpp>
#include <kinematics_interface/kinematics_interface/kinematics_interface.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <pluginlib/class_list_macros.hpp>

using namespace std::chrono_literals;

namespace delta_kinematics_controller
{

DeltaKinematicsController::DeltaKinematicsController() = default;

controller_interface::CallbackReturn DeltaKinematicsController::on_init()
{
  // default no-op
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DeltaKinematicsController::on_configure(const rclcpp_lifecycle::State &)
{
  // Declare and read parameters
  auto node = get_node();
  
  // Declare joints parameter
  if (!node->has_parameter("joints")) {
    node->declare_parameter("joints", std::vector<std::string>{});
  }
  joint_names_ = node->get_parameter("joints").as_string_array();

  // Declare lower_joints parameter (ball joint angles)
  if (!node->has_parameter("lower_joints")) {
    node->declare_parameter("lower_joints", std::vector<std::string>{});
  }
  
  try {
    lower_joint_names_ = node->get_parameter("lower_joints").as_string_array();
    if (!lower_joint_names_.empty()) {
      RCLCPP_INFO(node->get_logger(), "Loaded %zu lower joint names", lower_joint_names_.size());
    } else {
      RCLCPP_WARN(node->get_logger(), "No lower joints configured - only upper arm joints will be published");
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(node->get_logger(), "Failed to load lower_joints parameter: %s", e.what());
    lower_joint_names_.clear();
  }

  // Declare ee_joints parameter (prismatic ee_x, ee_y, ee_z)
  if (!node->has_parameter("ee_joints")) {
    node->declare_parameter("ee_joints", std::vector<std::string>{});
  }

  try {
    ee_joint_names_ = node->get_parameter("ee_joints").as_string_array();
    if (!ee_joint_names_.empty()) {
      RCLCPP_INFO(node->get_logger(), "Loaded %zu EE joint names (prismatic)", ee_joint_names_.size());
    } else {
      RCLCPP_WARN(node->get_logger(), "No ee_joints configured - EE will be published via TF only");
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(node->get_logger(), "Failed to load ee_joints parameter: %s", e.what());
    ee_joint_names_.clear();
  }

  // Declare kinematics_plugin_name parameter
  if (!node->has_parameter("kinematics_plugin_name")) {
    node->declare_parameter("kinematics_plugin_name", std::string(""));
  }
  kinematics_plugin_name_ = node->get_parameter("kinematics_plugin_name").as_string();

  // Declare base_link and ee_link parameters (optional, with defaults)
  if (!node->has_parameter("base_link")) {
    node->declare_parameter("base_link", "base_link");
  }
  base_link_ = node->get_parameter("base_link").as_string();

  if (!node->has_parameter("ee_link")) {
    node->declare_parameter("ee_link", "end_effector");
  }
  ee_link_ = node->get_parameter("ee_link").as_string();

  // Declare ee_tf_rate parameter (optional, default 50 Hz)
  if (!node->has_parameter("ee_tf_rate")) {
    node->declare_parameter("ee_tf_rate", 50.0);
  }
  ee_tf_rate_ = node->get_parameter("ee_tf_rate").as_double();

  // Declare geometry parameters for kinematics plugin (namespaced under kinematics_interface_delta)
  if (!node->has_parameter("kinematics_interface_delta.e")) {
    node->declare_parameter("kinematics_interface_delta.e", 0.045);
  }
  if (!node->has_parameter("kinematics_interface_delta.f")) {
    node->declare_parameter("kinematics_interface_delta.f", 0.11812);
  }
  if (!node->has_parameter("kinematics_interface_delta.re")) {
    node->declare_parameter("kinematics_interface_delta.re", 0.34);
  }
  if (!node->has_parameter("kinematics_interface_delta.rf")) {
    node->declare_parameter("kinematics_interface_delta.rf", 0.17438);
  }
  if (!node->has_parameter("kinematics_interface_delta.motor_z_offset")) {
    node->declare_parameter("kinematics_interface_delta.motor_z_offset", -0.025);
  }

  if (joint_names_.size() < 3)
  {
    RCLCPP_ERROR(node->get_logger(), "DeltaKinematicsController requires at least 3 joint names as parameter 'joints'");
    return controller_interface::CallbackReturn::ERROR;
  }

  // create plugin loader
  try
  {
    kinematics_loader_ = std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface_delta", "kinematics_interface::KinematicsInterface");
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create kinematics_interface loader: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // publisher for end-effector pose (computed from state interfaces)
  ee_pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", rclcpp::SystemDefaultsQoS());

  // publisher for joint states (absolute topic so robot_state_publisher receives them)
  joint_states_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SystemDefaultsQoS());

  // Create TF broadcaster if rate > 0
  if (ee_tf_rate_ > 0.0)
  {
    // create broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
    last_ee_tf_time_ = get_node()->now();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DeltaKinematicsController::on_activate(const rclcpp_lifecycle::State &)
{
  // load plugin instance
  try
  {
    kinematics_plugin_ = kinematics_loader_->createSharedInstance(kinematics_plugin_name_);
    if (!kinematics_plugin_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create kinematics plugin instance");
      return controller_interface::CallbackReturn::ERROR;
    }

    // initialize plugin with node parameters interface
    // initialize kinematics plugin with the configured ee_link name
    kinematics_plugin_->initialize(get_name(), get_node()->get_node_parameters_interface(), ee_link_);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception creating kinematics plugin: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DeltaKinematicsController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // release plugin
  kinematics_plugin_.reset();
  kinematics_loader_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DeltaKinematicsController::command_interface_configuration() const
{
  // This controller only monitors joint states and computes FK
  // It does not command any joints
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

controller_interface::InterfaceConfiguration DeltaKinematicsController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto &j : joint_names_)
    conf.names.push_back(j + "/position");
  return conf;
}

controller_interface::return_type DeltaKinematicsController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  (void)time;
  (void)period;
  
  // read current joint positions
  Eigen::VectorXd joint_angles(3);
  for (size_t i = 0; i < 3; ++i)
  {
    joint_angles(i) = this->state_interfaces_[i].get_value();
  }

  // calculate and publish end-effector transform using the position state interfaces
  if (kinematics_plugin_)
  {
    Eigen::Isometry3d ee_tf = Eigen::Isometry3d::Identity();
    Eigen::VectorXd joint_angles_eig = joint_angles;
    
    RCLCPP_INFO_ONCE(get_node()->get_logger(), "Before FK: ee_tf translation=[%.6f, %.6f, %.6f]",
                     ee_tf.translation().x(), ee_tf.translation().y(), ee_tf.translation().z());
    
    // FK: compute end-effector transform
    bool fk_success = kinematics_plugin_->calculate_link_transform(joint_angles_eig, ee_link_, ee_tf);
    
    RCLCPP_INFO_ONCE(get_node()->get_logger(), "After FK: success=%d ee_tf translation=[%.6f, %.6f, %.6f]",
                     fk_success, ee_tf.translation().x(), ee_tf.translation().y(), ee_tf.translation().z());
    
    if (fk_success)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = get_node()->now();
      pose.header.frame_id = base_link_;
      Eigen::Vector3d t = ee_tf.translation();
      pose.pose.position.x = t.x();
      pose.pose.position.y = t.y();
      pose.pose.position.z = t.z();
      Eigen::Quaterniond q(ee_tf.rotation());
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      if (ee_pose_pub_) ee_pose_pub_->publish(pose);
      
      // Debug output (first 5 times)
      static int debug_count = 0;
      if (debug_count < 5) {
        RCLCPP_INFO(get_node()->get_logger(), "FK SUCCESS: joints=[%.3f, %.3f, %.3f] -> EE pos=[%.4f, %.4f, %.4f]",
                    joint_angles(0), joint_angles(1), joint_angles(2), t.x(), t.y(), t.z());
        debug_count++;
      }

      // Calculate elbow positions geometrically from joint angles
      // Get delta robot geometry from ROS parameters (loaded in on_configure)
      const double delta_rf = get_node()->get_parameter("kinematics_interface_delta.rf").as_double();
      const double delta_e = get_node()->get_parameter("kinematics_interface_delta.e").as_double();
      const double delta_f = get_node()->get_parameter("kinematics_interface_delta.f").as_double();
      const double motor_z_offset = get_node()->get_parameter("kinematics_interface_delta.motor_z_offset").as_double();
      
      std::array<std::array<double,3>, 3> elbows;
      
      // Calculate 't' parameter: base radius - EE radius
      const double sin30 = 0.5;
      const double cos30 = std::cos(M_PI / 6.0);
      double base_t = delta_f - delta_e;  // e,f are radii directly
      
      // Calculate elbow positions matching FK coordinate convention
      // FK: arm 1 at -Y, arm 2 at 30° from +X, arm 3 at 150° from +X
      
      // Elbow 1 (extends in -Y direction)
      elbows[0][0] = 0.0;
      elbows[0][1] = -(base_t + delta_rf * std::cos(joint_angles(0)));
      elbows[0][2] = -delta_rf * std::sin(joint_angles(0)) + motor_z_offset;
      
      // Elbow 2 (radial direction at 30° from +X)
      elbows[1][0] = (base_t + delta_rf * std::cos(joint_angles(1))) * cos30;   // x = cos30 * (...)
      elbows[1][1] = (base_t + delta_rf * std::cos(joint_angles(1))) * sin30;   // y = sin30 * (...)
      elbows[1][2] = -delta_rf * std::sin(joint_angles(1)) + motor_z_offset;
      
      // Elbow 3 (radial direction at 150° from +X)
      elbows[2][0] = -(base_t + delta_rf * std::cos(joint_angles(2))) * cos30;  // x = -cos30 * (...)
      elbows[2][1] = (base_t + delta_rf * std::cos(joint_angles(2))) * sin30;   // y = sin30 * (...)
      elbows[2][2] = -delta_rf * std::sin(joint_angles(2)) + motor_z_offset;

      // Calculate passive ball joint angles (pitch and yaw) for each lower arm
      // URDF joint chain: Ry(pitch) then Rz(yaw), lower arm visual extends in -X
      // The URDF arm yaw angles define each arm's radial direction
      const double arm_yaw[3] = {-M_PI/2.0, M_PI/6.0, 5.0*M_PI/6.0};  // matches URDF xacro
      
      // EE platform attachment point offsets from EE center.
      // e is the radius from EE center to rod-end attachment point.
      // Arm 1 attaches at -Y, arm 2 at 120° CW, arm 3 at 240° CW (matching FK convention).
      const double wp = delta_e;  // e is already the radius
      // Attachment angles in FK convention: arm1=-90°, arm2=30°, arm3=150°
      const double attach_angle[3] = {-M_PI/2.0, M_PI/6.0, 5.0*M_PI/6.0};
      double ee_attach[3][3];
      for (int i = 0; i < 3; ++i) {
        ee_attach[i][0] = t.x() + wp * std::cos(attach_angle[i]);
        ee_attach[i][1] = t.y() + wp * std::sin(attach_angle[i]);
        ee_attach[i][2] = t.z();  // EE platform is horizontal
      }
      
      std::array<double, 6> lower_arm_angles;
      for (int i = 0; i < 6; ++i) lower_arm_angles[i] = 0.0;
      
      for (int i = 0; i < 3; ++i)
      {
        // Direction vector from elbow to EE platform attachment point (not center!)
        double dx_g = ee_attach[i][0] - elbows[i][0];
        double dy_g = ee_attach[i][1] - elbows[i][1];
        double dz_g = ee_attach[i][2] - elbows[i][2];
        double length = std::sqrt(dx_g*dx_g + dy_g*dy_g + dz_g*dz_g);
        if (length > 1e-6) { dx_g /= length; dy_g /= length; dz_g /= length; }
        
        // Transform direction into the arm's radial frame: Rz(-arm_yaw)
        // This aligns with the URDF joint frame before motor rotation
        double ca = std::cos(arm_yaw[i]);
        double sa = std::sin(arm_yaw[i]);
        double dx_arm =  ca * dx_g + sa * dy_g;
        double dy_arm = -sa * dx_g + ca * dy_g;
        double dz_arm = dz_g;
        
        // In the arm frame, the lower arm visual extends along -X.
        // The full elbow frame = Rz(arm_yaw) * Ry(motor_angle).
        // After pitch+yaw joints: Ry(motor + pitch) * Rz(yaw) * [-1,0,0] = (dx_arm, dy_arm, dz_arm)
        //
        // Solving:
        //   combined_pitch = atan2(dz_arm, -dx_arm)
        //   yaw = -atan2(dy_arm, sqrt(dx_arm² + dz_arm²))
        //   pitch_joint = combined_pitch - motor_angle
        
        double motor_angle = joint_angles(i);
        double cos_phi = std::sqrt(dx_arm*dx_arm + dz_arm*dz_arm);
        double combined_pitch = std::atan2(dz_arm, -dx_arm);
        double yaw = -std::atan2(dy_arm, cos_phi);
        double pitch = combined_pitch - motor_angle;
        
        lower_arm_angles[i*2] = pitch;
        lower_arm_angles[i*2 + 1] = yaw;
      }

      // Publish ONLY the EE transform via TF (robot_state_publisher handles arm links from joint_states)
      // When ee_joints are configured (prismatic chain), publish EE position as joint_states instead
      // so robot_state_publisher computes the TF from the prismatic chain.
      if (tf_broadcaster_ && ee_tf_rate_ > 0.0)
      {
        rclcpp::Time now = get_node()->now();
        double tf_period = 1.0 / ee_tf_rate_;
        if ((now - last_ee_tf_time_).seconds() >= tf_period)
        {
          // Publish joint states at same rate as EE
          if (joint_states_pub_)
          {
            sensor_msgs::msg::JointState js_msg;
            js_msg.header.stamp = now;
            js_msg.header.frame_id = base_link_;
            
            // Publish lower arm ball joints (pitch and yaw for each arm)
            if (!lower_joint_names_.empty())
            {
              for (const auto& name : lower_joint_names_)
              {
                js_msg.name.push_back(name);
              }
              for (size_t i = 0; i < lower_arm_angles.size() && i < lower_joint_names_.size(); ++i)
              {
                js_msg.position.push_back(lower_arm_angles[i]);
              }
            }

            // Publish EE prismatic joint positions (ee_x, ee_y, ee_z)
            if (ee_joint_names_.size() >= 3)
            {
              js_msg.name.push_back(ee_joint_names_[0]);  // ee_x
              js_msg.position.push_back(t.x());
              js_msg.name.push_back(ee_joint_names_[1]);  // ee_y
              js_msg.position.push_back(t.y());
              js_msg.name.push_back(ee_joint_names_[2]);  // ee_z
              js_msg.position.push_back(t.z());
            }

            joint_states_pub_->publish(js_msg);
          }

          // Only broadcast TF for EE if prismatic joints are NOT configured
          // (when prismatic joints exist, robot_state_publisher handles EE from joint_states)
          if (ee_joint_names_.empty())
          {
            geometry_msgs::msg::TransformStamped tmsg;
            tmsg.header.stamp = now;
            tmsg.header.frame_id = base_link_;
            tmsg.child_frame_id = ee_link_;
            tmsg.transform.translation.x = t.x();
            tmsg.transform.translation.y = t.y();
            tmsg.transform.translation.z = t.z();
            tmsg.transform.rotation.x = q.x();
            tmsg.transform.rotation.y = q.y();
            tmsg.transform.rotation.z = q.z();
            tmsg.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(tmsg);
          }

          // Elbow/lower arm TF publishing disabled - robot_state_publisher handles those from joint_states
          // (pitch and yaw joints are published to /joint_states, robot_state_publisher positions lower_link)
          /*
          // Calculate and publish elbow positions and lower arm transforms
          // Based on delta robot geometry from the kinematics plugin parameters
          // Note: delta_rf already defined above for elbow calculations
          
          // Base attachment points (120 degrees apart)
          std::vector<double> base_angles = {0.0, 2.09439510239, 4.18879020478};
          double base_radius = 0.06;
          
          for (size_t i = 0; i < 3; ++i)
          {
            // Calculate elbow position based on upper arm angle
            double theta = joint_angles(i);
            double base_x = base_radius * std::cos(base_angles[i]);
            double base_y = base_radius * std::sin(base_angles[i]);
            double base_z = -0.025;  // motor_z_offset
            
            // Elbow position (end of upper arm)
            double elbow_x = base_x + delta_rf * std::cos(base_angles[i]) * std::cos(theta);
            double elbow_y = base_y + delta_rf * std::sin(base_angles[i]) * std::cos(theta);
            double elbow_z = base_z - delta_rf * std::sin(theta);
            
            // Vector from elbow to end-effector
            Eigen::Vector3d elbow_pos(elbow_x, elbow_y, elbow_z);
            Eigen::Vector3d lower_arm_vec = t - elbow_pos;
            
            // Calculate orientation of lower arm (points from elbow to EE)
            Eigen::Vector3d z_axis = lower_arm_vec.normalized();
            
            // Choose perpendicular vector for x-axis
            Eigen::Vector3d x_axis;
            if (std::abs(z_axis.z()) < 0.9) {
              x_axis = Eigen::Vector3d(0, 0, 1).cross(z_axis).normalized();
            } else {
              x_axis = Eigen::Vector3d(1, 0, 0).cross(z_axis).normalized();
            }
            Eigen::Vector3d y_axis = z_axis.cross(x_axis);
            
            Eigen::Matrix3d lower_arm_rot;
            lower_arm_rot.col(0) = x_axis;
            lower_arm_rot.col(1) = y_axis;
            lower_arm_rot.col(2) = z_axis;
            Eigen::Quaterniond lower_arm_quat(lower_arm_rot);
            
            // Publish elbow TF
            geometry_msgs::msg::TransformStamped elbow_tf;
            elbow_tf.header.stamp = now;
            elbow_tf.header.frame_id = base_link_;
            elbow_tf.child_frame_id = "elbow_" + std::to_string(i + 1);
            elbow_tf.transform.translation.x = elbow_x;
            elbow_tf.transform.translation.y = elbow_y;
            elbow_tf.transform.translation.z = elbow_z;
            elbow_tf.transform.rotation.w = 1.0;
            elbow_tf.transform.rotation.x = 0.0;
            elbow_tf.transform.rotation.y = 0.0;
            elbow_tf.transform.rotation.z = 0.0;
            tf_broadcaster_->sendTransform(elbow_tf);
            
            // Publish lower arm TF (from elbow to midpoint of lower arm)
            geometry_msgs::msg::TransformStamped lower_arm_tf;
            lower_arm_tf.header.stamp = now;
            lower_arm_tf.header.frame_id = "elbow_" + std::to_string(i + 1);
            lower_arm_tf.child_frame_id = "lower_link_" + std::to_string(i + 1);
            lower_arm_tf.transform.translation.x = 0.0;
            lower_arm_tf.transform.translation.y = 0.0;
            lower_arm_tf.transform.translation.z = 0.0;
            lower_arm_tf.transform.rotation.x = lower_arm_quat.x();
            lower_arm_tf.transform.rotation.y = lower_arm_quat.y();
            lower_arm_tf.transform.rotation.z = lower_arm_quat.z();
            lower_arm_tf.transform.rotation.w = lower_arm_quat.w();
            tf_broadcaster_->sendTransform(lower_arm_tf);
          }
          */  // End of disabled TF publishing block
          
          last_ee_tf_time_ = now;
        }
      }
    }  // End of FK success block
    else
    {
      static int error_count = 0;
      if (error_count < 5) {
        RCLCPP_ERROR(get_node()->get_logger(), "Forward kinematics failed for joints: [%.3f, %.3f, %.3f]", 
                     joint_angles(0), joint_angles(1), joint_angles(2));
        error_count++;
      }
    }
  }

  return controller_interface::return_type::OK;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(
  delta_kinematics_controller::DeltaKinematicsController,
  controller_interface::ControllerInterface)
