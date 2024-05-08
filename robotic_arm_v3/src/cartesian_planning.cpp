#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robotic_arm_v3");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("cartesian_planning", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Adding Collision Object

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  collision_object.id = "box1";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 0.2;
  primitive.dimensions[primitive.BOX_Z] = 1.5;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.48;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.7;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Cartesian Planning towards Position 1

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose1;

  target_pose1.orientation.x = 0.5;
  target_pose1.orientation.y = -0.5;
  target_pose1.orientation.z = -0.5;
  target_pose1.orientation.w = 0.5;
  target_pose1.position.x = 0.573;
  target_pose1.position.y = 0.456;
  target_pose1.position.z = 1.363;
  waypoints.push_back(target_pose1); 

  target_pose1.position.z = 1.663;          // An extra point to move around the Object
  waypoints.push_back(target_pose1); 

  // move_group.setPoseTarget(target_pose1);

  target_pose1.position.y = -0.566;      // Point 2 for linear transilation

  waypoints.push_back(target_pose1); 

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  move_group.execute(trajectory);

  // Back to Home
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan; 

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan);

  // RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//   if(success) {
//   move_group.execute(my_plan);
// } else {
//   RCLCPP_ERROR(LOGGER, "Planing failed!");
// }

  rclcpp::shutdown();
  return 0;
}