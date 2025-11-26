#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";

class PickAndPlace {
public:
  PickAndPlace(rclcpp::Node::SharedPtr base_node) : base_node_(base_node) {
    RCLCPP_INFO(LOGGER, "Initialing Class: Pick And Place");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    RCLCPP_INFO(LOGGER, "Initialized Class: Pick And Place");
  }

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;

  rclcpp::Node::SharedPtr base_node_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place");
  PickAndPlace pick_and_place(base_node);

  rclcpp::shutdown();
  return 0;
}