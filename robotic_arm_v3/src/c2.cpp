#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
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

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "J1_URDFT", "move_group_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^

  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz:
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "1Feet";
  ocm.header.frame_id = "J1_URDFT";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // Enforce Planning in Joint Space
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Depending on the planning problem MoveIt chooses between
  // ``joint space`` and ``cartesian space`` for problem representation.
  // Setting the group parameter ``enforce_joint_model_state_space:true`` in
  // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
  //
  // By default, planning requests with orientation path constraints
  // are sampled in ``cartesian space`` so that invoking IK serves as a
  // generative sampler.
  //
  // By enforcing ``joint space``, the planning process will use rejection
  // sampling to find valid requests. Please note that this might
  // increase planning time considerably.
  //
  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So we need to set the start
  // state to a new pose.
  moveit::core::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::msg::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // Now, we will plan to the earlier pose target from the new
  // start state that we just created.
  move_group.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Let's increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz:
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "home");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // When done with the path constraint, be sure to clear it.
  move_group.clearPathConstraints();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::msg::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

  // First, let's plan to another simple goal with no objects in the way.
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::msg::Pose another_pose;
  another_pose.orientation.w = 0;
  another_pose.orientation.x = -1.0;
  another_pose.position.x = 0.7;
  another_pose.position.y = 0.0;
  another_pose.position.z = 0.59;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

  if(success) {
  move_group.execute(my_plan);
} else {
  RCLCPP_ERROR(LOGGER, "Planing failed!");
}

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Clear_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishAxisLabeled(another_pose, "goal");
  visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
  visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // The result may look like this:
  //
  // .. image:: ./move_group_interface_tutorial_clear_path.gif
  //    :alt: animation showing the arm moving relatively straight toward the goal
  //
  rclcpp::shutdown();
  return 0;
}