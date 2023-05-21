//
// Created by lsy on 23-3-7.
//

#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

// moveit msgs
#include <moveit_msgs/MotionSequenceResponse.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupSequenceActionGoal.h>
#include <moveit_msgs/MoveGroupSequenceAction.h>
#include <moveit_msgs/MoveGroupSequenceActionResult.h>

int main(int argc, char** argv)
{
  // ROS COMMON
  ros::init(argc, argv, "pilz_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // ROBOT MODEL
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  // MOVE GROUP INTERFACE
  moveit::planning_interface::MoveGroupInterface move_group_interface_("engineer_arm");

  // PLANNING SCENE
  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();

  // ROBOT SATE
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  // JOINT MODEL
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("engineer_arm");

  // plugin
  //  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  //  planning_interface::PlannerManagerPtr planner_instance;
  //  std::string planner_plugin_name = "pilz_industrial_motion_planner::CommandPlanner";
  //
  //  try
  //  {
  //    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
  //        "moveit_core", "planning_interface::PlannerManager"));
  //  }
  //  catch (pluginlib::PluginlibException& ex)
  //  {
  //    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  //  }
  //  try
  //  {
  //    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  //    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
  //      ROS_FATAL_STREAM("Could not initialize planner instance");
  //    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  //  }
  //  catch (pluginlib::PluginlibException& ex)
  //  {
  //    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
  //    std::stringstream ss;
  //    for (const auto& cls : classes)
  //      ss << cls << " ";
  //    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
  //                                                         << "Available plugins: " << ss.str());
  //  }
  //
  //  planning_pipeline::PlanningPipelinePtr planning_pipeline(
  //      new planning_pipeline::PlanningPipeline(robot_model, node_handle, "chomp_interface/CHOMPPlanner",
  //                                              "default_planner_request_adapters/FixStartStateCollision"));
  //
  //  ROS_INFO_STREAM("PIPELINE" << planning_pipeline->getPlannerPluginName());
  // planning_interface::MotionPlanRequest req;
  moveit_msgs::MotionPlanRequest req;
  moveit_msgs::MotionSequenceRequest reqs;
  // planning_interface::MotionPlanResponse res;
  // Initialize the assignment
  // req.planner_id = "PTP";
  req.group_name = "engineer_arm";
  // req.max_acceleration_scaling_factor = max_acceleration_;
  req.max_acceleration_scaling_factor = 2;
  // req.max_velocity_scaling_factor = velocity_factor_;
  req.max_velocity_scaling_factor = 1;
  // start_state joints_state
  moveit::core::RobotState goal_state(*robot_state);
  std::vector<double> joint_values = { 0.276, 0.013, -0.276, -1.541, 1.656, 0.007 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
  // planning_pipeline->generatePlan(lscene, req, res);
  /* Check that the planning was successful */
  //  if (res.error_code_.val != res.error_code_.SUCCESS)
  //  {
  //    ROS_ERROR("Could not compute plan successfully");
  //    return 0;
  //  }
  //  moveit_msgs::MotionPlanResponse response;
  //  res.getMessage(response);
  //  ROS_INFO_STREAM("RESPONSE");
  // move_group_interface_.execute(response.trajectory);

  // create a client to connect to the sequence_move_group action server
  actionlib::SimpleActionClient<moveit_msgs::MoveGroupSequenceAction> client("/sequence_move_group", true);
  // wait for the server to start
  ROS_INFO("Waiting for sequence_move_group action server to start...");
  client.waitForServer();
  ROS_INFO("Connection");
  // create a request message
  moveit_msgs::MoveGroupSequenceActionGoalPtr goal;
  // fill in the request fields here...

  // send the request to the server and wait for the response
  ROS_INFO("Sending motion sequence request to sequence_move_group action server...");
  moveit_msgs::MotionSequenceItem req_item;
  req_item.req = req;
  req_item.blend_radius = 0.01;
  goal->goal.request.items.push_back(req_item);
  client.sendGoal(goal->goal);
  client.waitForResult();
  moveit_msgs::MoveGroupSequenceResultConstPtr result;
  // handle the response
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    result = client.getResult();
    // process the result here...
    ROS_INFO_STREAM("DONN");
  }
  else
  {
    ROS_ERROR("Failed to receive motion sequence result from sequence_move_group action server.");
  }
  return 0;
}
