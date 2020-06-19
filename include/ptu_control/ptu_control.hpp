#ifndef PTU_CONTROL_H
#define PTU_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <urdf/model.h>


#include <chrono>
#include <string.h>

#include <geometry_msgs/msg/twist.hpp>

#include <rover_msgs/msg/joint_command.hpp>
#include <rover_msgs/msg/joint_command_array.hpp>

using namespace std::chrono_literals;

class PTUControl : public rclcpp::Node
{
public:
  PTUControl(rclcpp::NodeOptions options, std::string node_name);

private:
  
  void load_params();

  void load_ptu_model();

  // Access parameters through the parameters_client_
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;


  // PTU Velocity Command Subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ptu_velocities_subscription_;
  // TODO: PTU Position Command Action Server including feedback

  // Joint command publisher
  rclcpp::Publisher<rover_msgs::msg::JointCommandArray>::SharedPtr joint_command_publisher_;

  // Velocity Callback
  void ptu_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Robot model that contains the PTU
  std::shared_ptr<urdf::Model> model_;

  // URDF Model
  std::string model_name_;
  std::string model_dir_;
  std::string model_path_;


  // Identifiers specified in config file
  std::string pan_joint_identifier_;
  std::string tilt_joint_identifier_;

  std::shared_ptr<urdf::Joint> pan_joint_;
  std::shared_ptr<urdf::Joint> tilt_joint_;

  std::vector<std::shared_ptr<urdf::Joint>> joints_;
  std::vector<std::shared_ptr<urdf::Link>> links_;


};


#endif
