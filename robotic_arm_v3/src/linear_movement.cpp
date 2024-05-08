#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robotic_arm_v3");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  // void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("linear_movement", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}


void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /* max_solutions */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_frame = "END_EFFECTOR";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Set a target Point_1
  //   pose:
    //     position:
    //       x: 0.9933247753803834
    //       y: -0.40575783877915145
    //       z: 0.9233955123696119
    //     orientation:
    //       x: 0.5071103419129587
    //       y: 0.47722876590108765
    //       z: 0.5087603689537635
    //       w: 0.5062160537788953
  auto const point_1 = []
    {
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.orientation.w = 0.5;
    msg.pose.position.x = 1.0;
    msg.pose.position.y = -0.4;
    msg.pose.position.z = 1.0;
    return msg;
    }();

  // Set a target Point_2
    // pose:
    //     position:
    //       x: 0.9902977215670196
    //       y: 0.7160172344355493
    //       z: 0.910649849698846
    //     orientation:
    //       x: 0.5071137715860139
    //       y: 0.47723145519449883
    //       z: 0.5087570818844291
    //       w: 0.5062133863038288


  auto const point_2 = []
    {
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.orientation.w = 0.5;
    msg.pose.position.x = 1.0;
    msg.pose.position.y = 0.7;
    msg.pose.position.z = 1.0;
    return msg;
    }();

  {
    // Move to point_1
    auto stage_move_to_point_1 = std::make_unique<mtc::stages::MoveTo>("move to point_1",interpolation_planner);
    stage_move_to_point_1->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame"});
    stage_move_to_point_1->setGoal(point_1);
    task.add(std::move(stage_move_to_point_1));
  }

  {
    // Move to point_2
    auto stage_move_to_point_2 = std::make_unique<mtc::stages::MoveTo>("move to point_2", interpolation_planner);
    stage_move_to_point_2->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame"});
    stage_move_to_point_2->setGoal(point_2);
    task.add(std::move(stage_move_to_point_2));
  }

  {
    // Return Home
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("home");
    task.add(std::move(stage));
  }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}