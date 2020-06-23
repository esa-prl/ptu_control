#include "ptu_control/ptu_control.hpp"

PTUControl::PTUControl(rclcpp::NodeOptions options, std::string node_name)
: Node(node_name,
    options.allow_undeclared_parameters(true).
    automatically_declare_parameters_from_overrides(true)),
  parameters_client_(std::make_shared<rclcpp::SyncParametersClient>(this)),
  model_(new urdf::Model())
{
  // Load Params
  load_params();

  // Load URDF
  if (!load_ptu_model()) {
    RCLCPP_ERROR(this->get_logger(), "TODO: Add Proper error handling for this shutdown RIP node.");
  }

  // Create Publishers
  joint_command_publisher_ = this->create_publisher<rover_msgs::msg::JointCommandArray>(
    "joint_cmds", 10);

  // // Create Subscriptions
  // joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
  //   "joint_states", 10, std::bind(
  //     &LocomotionMode::joint_state_callback, this,
  //     std::placeholders::_1));

  ptu_velocities_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "ptu_cmd", 10,
    std::bind(&PTUControl::ptu_velocities_callback, this, std::placeholders::_1));


  RCLCPP_INFO(this->get_logger(), "PTUControl initialized");
}


void PTUControl::load_params()
{
  while (!parameters_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  // Load urdf model path
  model_path_ = parameters_client_->get_parameters({"robot_description"})[0].value_to_string();

  pan_joint_identifier_ = parameters_client_->get_parameters({"pan_joint_identifier"})[0].value_to_string();
  tilt_joint_identifier_ = parameters_client_->get_parameters({"tilt_joint_identifier"})[0].value_to_string();
}


bool PTUControl::load_ptu_model()
{
  if (!model_->initFile(model_path_)) {
    RCLCPP_ERROR(
      this->get_logger(), "URDF file [%s] not found. Make sure the path is specified in the launch file.",
      model_path_.c_str());
  } else {RCLCPP_INFO(this->get_logger(), "Successfully parsed urdf file.");}

  // Get Links
  model_->getLinks(links_);

  for (std::shared_ptr<urdf::Link> link : links_) {
    // Get Joints
    if (link->child_joints.size() != 0) {
      for (std::shared_ptr<urdf::Joint> child_joint : link->child_joints) {
        joints_.push_back(child_joint);

        if (child_joint->name.find(pan_joint_identifier_) != std::string::npos) {
          if (child_joint->type == urdf::Joint::REVOLUTE ||
            child_joint->type == urdf::Joint::CONTINUOUS)
          {
            pan_joint_ = child_joint;
            RCLCPP_INFO(this->get_logger(), "PAN JOINT FOUND!");
          }
        }
        else if (child_joint->name.find(tilt_joint_identifier_) != std::string::npos) {
          if (child_joint->type == urdf::Joint::REVOLUTE ||
            child_joint->type == urdf::Joint::CONTINUOUS)
          {
            tilt_joint_ = child_joint;
            RCLCPP_INFO(this->get_logger(), "TILT JOINT FOUND!");
          }
        }
      }
    }
  }

  if (!pan_joint_) {
    RCLCPP_WARN(this->get_logger(), "NO PAN JOINT FOUND!");
  }

  if (!tilt_joint_) {
    RCLCPP_WARN(this->get_logger(), "NO TILT JOINT FOUND!");
  }

  if (!pan_joint_ && !tilt_joint_) {
    RCLCPP_ERROR(this->get_logger(), "NO PAN NOR TILT JOINT FOUND! ABORTING!");
    return false;
  }

  return true;
}

void PTUControl::ptu_velocities_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // TODO: CHECK FOR LIMITS


  // TODO: Make clock member variable
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // Create JointCommandArray Msg
  rover_msgs::msg::JointCommandArray joint_command_array_msg;

  // Create Pan and Tilt Joint Message
  rover_msgs::msg::JointCommand pan_msg;
  rover_msgs::msg::JointCommand tilt_msg;

  if (pan_joint_) {
    // Fills Driving Message
    pan_msg.header.stamp = clock->now();
    pan_msg.name = pan_joint_->name;
    pan_msg.mode = ("VELOCITY");
    pan_msg.value = msg->angular.z;
  
    joint_command_array_msg.joint_command_array.push_back(pan_msg);
  }

  if (tilt_joint_) {

    tilt_msg.header.stamp = clock->now();
    tilt_msg.name = tilt_joint_->name;
    tilt_msg.mode = ("VELOCITY");
    tilt_msg.value = msg->angular.y;
    
    joint_command_array_msg.joint_command_array.push_back(tilt_msg);
  }


  joint_command_publisher_->publish(joint_command_array_msg);
}




int main(int argc, char * argv[])
{
  rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PTUControl>(options, "ptu_control_node"));
  rclcpp::shutdown();
  return 0;
}
