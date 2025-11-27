#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlace {
public:
  PickAndPlace(rclcpp::Node::SharedPtr base_node) : base_node_(base_node) {
    RCLCPP_INFO(LOGGER, "Initialing Class: Pick And Place");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // Init move group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);
    // Get init state
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);
    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);
    // Set start state of robot to current state
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    RCLCPP_INFO(LOGGER, "Initialized Class: Pick And Place");
  }

  ~PickAndPlace() { RCLCPP_INFO(LOGGER, "Class Terminated"); }

  void execute_trajectory() {
    RCLCPP_INFO(LOGGER, "Executing Pick And Place");

    // setup_joint_value_target(+0.0000, -2.3562, +1.5708, -1.5708, -1.5708,
    //                          +0.0000);
    setup_goal_pose_target(+0.343, +0.132, +0.284, -1.000, +0.000, +0.000,
                           +0.000);
    execute_plan(move_group_robot_, kinematics_trajectory_plan_,
                 "Goal Pose Trajectory");

    setup_joint_value_gripper(0.4);
    execute_plan(move_group_gripper_, gripper_trajectory_plan_,
                 "Gripper Action");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    setup_joint_value_gripper(0.8);
    execute_plan(move_group_gripper_, gripper_trajectory_plan_,
                 "Gripper Action");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    RCLCPP_INFO(LOGGER, "Complate");
  }

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  // trajectory planning variables for robot
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;

  // trajectory planning variables for gripper
  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void setup_joint_value_gripper(float angle) {
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  //   void setup_named_pose_gripper(std::string pose_name) {
  //     move_group_gripper_->setNamedTarget(pose_name);
  //   }

  bool execute_plan(std::shared_ptr<MoveGroupInterface> move_group, Plan &plan,
                    const std::string plan_type) {
    RCLCPP_INFO(LOGGER, "Planning %s", plan_type.c_str());

    bool plan_success =
        (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!plan_success) {
      RCLCPP_ERROR(LOGGER, "Failed planning %s", plan_type.c_str());
      return false;
    }

    RCLCPP_INFO(LOGGER, "Executing %s", plan_type.c_str());
    bool result =
        (move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!result) {
      RCLCPP_ERROR(LOGGER, "Failed executing %s", plan_type.c_str());
      return false;
    }

    RCLCPP_INFO(LOGGER, "Succeeded %s", plan_type.c_str());
    return true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place");
  PickAndPlace pick_and_place(base_node);
  pick_and_place.execute_trajectory();

  rclcpp::shutdown();
  return 0;
}