/**
 *  @file   traj.cpp
 *  @brief  trajectory: action client, initialize, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 * 
 *  @details ROS2 modifications made by by Mikolaj Kliniewski Oct 3, 2024
 **/

#include "traj.h"
#include <rclcpp/rclcpp.hpp>

namespace piper {

/* ************************************************************************** */
Traj::Traj(rclcpp::Node::SharedPtr node)
{
  // ROS trajectory joint names
  node->get_parameter("robot.arm_joint_names", arm_joint_names);
  traj_.trajectory.joint_names = arm_joint_names;

  // to visualize estimated trajectory
  if (node->has_parameter("robot.est_traj_pub_topic"))
  {
    node->get_parameter("robot.est_traj_pub_topic", est_traj_pub_topic_);
    est_traj_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(est_traj_pub_topic_, 1);
  }

  // to visualize planned trajectory
  if (node->has_parameter("robot.plan_traj_pub_topic"))
  {
    node->get_parameter("robot.plan_traj_pub_topic", plan_traj_pub_topic_);
    plan_traj_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(plan_traj_pub_topic_, 1);
  }

  // trajectory action client
  if (node->has_parameter("robot.trajectory_control_topic"))
  {
    node->get_parameter("robot.trajectory_control_topic", trajectory_control_topic_);
    traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node, trajectory_control_topic_);
    if (!traj_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_INFO(node->get_logger(), "Waiting for trajectory_control server...");
      if (!traj_client_->wait_for_action_server(std::chrono::seconds(5)))
      {
        RCLCPP_ERROR(node->get_logger(), "Cannot find trajectory_control server '%s'", trajectory_control_topic_.c_str());
        RCLCPP_FATAL(node->get_logger("piper"), "Quitting...");
        sigintHandler(0)
      }
    }
  }
  else
    RCLCPP_WARN(node->get_logger(), "No trajectory control topic. Trajectory will not be executed.");
}

/* ************************************************************************** */
void Traj::initializeTrajectory(gtsam::Values& init_values, Problem& problem)
{
  RCLCPP_INFO(rclcpp::get_logger("piper"), "Initializing trajectory.");
  gtsam::Vector conf, avg_vel;
  if (!problem.robot.isMobileBase())
  {
    avg_vel = (problem.goal_conf - problem.start_conf) / problem.total_time;
    for (std::size_t i = 0; i < problem.total_step; i++)
    {
      double ratio = static_cast<double>(i) / static_cast<double>(problem.total_step - 1);
      conf = (1.0 - ratio) * problem.start_conf + ratio * problem.goal_conf;
      init_values.insert(gtsam::Symbol('x', i), conf);
      init_values.insert(gtsam::Symbol('v', i), avg_vel);
    }
  }
  else
  {
    gtsam::Pose2 pose;
    avg_vel = (gtsam::Vector(problem.robot.getDOF()) << problem.goal_pose.x() - problem.start_pose.x(),
                problem.goal_pose.y() - problem.start_pose.y(),
                problem.goal_pose.theta() - problem.start_pose.theta(),
                problem.goal_conf - problem.start_conf).finished() / problem.total_time;
    for (std::size_t i = 0; i < problem.total_step; i++)
    {
      double ratio = static_cast<double>(i) / static_cast<double>(problem.total_step - 1);
      pose = gtsam::Pose2((1.0 - ratio) * problem.start_pose.x() + ratio * problem.goal_pose.x(),
                          (1.0 - ratio) * problem.start_pose.y() + ratio * problem.goal_pose.y(),
                          (1.0 - ratio) * problem.start_pose.theta() + ratio * problem.goal_pose.theta());
      conf = (1.0 - ratio) * problem.start_conf + ratio * problem.goal_conf;
      init_values.insert(gtsam::Symbol('x', i), gpmp2::Pose2Vector(pose, conf));
      init_values.insert(gtsam::Symbol('v', i), avg_vel);
    }
  }
}

/* ************************************************************************** */
void Traj::executeTrajectory(gtsam::Values& exec_values, Problem& problem, size_t exec_step)
{
  gtsam::Pose2 pose;
  gtsam::Vector conf, vel;
  int DOF = problem.robot.getDOF();
  int DOF_arm = problem.robot.getDOFarm();

  // Create ROS trajectory
  traj_.trajectory.points.resize(exec_step);
  for (std::size_t i = 0; i < exec_step; i++)
  {
    if (!problem.robot.isMobileBase())
    {
      traj_.trajectory.points[i].positions.resize(DOF_arm);
      traj_.trajectory.points[i].velocities.resize(DOF_arm);
      conf = exec_values.at<gtsam::Vector>(gtsam::Symbol('x', i));
      vel = exec_values.at<gtsam::Vector>(gtsam::Symbol('v', i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      for (std::size_t j = 0; j < DOF_arm; j++)
      {
        traj_.trajectory.points[i].positions[j] = conf[j];
        traj_.trajectory.points[i].velocities[j] = vel[j];
      }
    }
    else
    {
      traj_.trajectory.points[i].positions.resize(DOF);
      traj_.trajectory.points[i].velocities.resize(DOF);
      pose = exec_values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i)).pose();
      conf = exec_values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i)).configuration();
      vel = exec_values.at<gtsam::Vector>(gtsam::Symbol('v', i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      traj_.trajectory.points[i].positions[0] = pose.x();
      traj_.trajectory.points[i].positions[1] = pose.y();
      traj_.trajectory.points[i].positions[2] = pose.theta();
      for (std::size_t j = 0; j < DOF_arm; j++)
        traj_.trajectory.points[i].positions[j + 3] = conf[j];
      for (std::size_t j = 0; j < DOF; j++)
        traj_.trajectory.points[i].velocities[j] = vel[j];
    }
    traj_.trajectory.points[i].time_from_start = rclcpp::Duration(i * problem.delta_t / (problem.control_inter + 1));
  }
  traj_.trajectory.header.stamp = rclcpp::Clock().now();

  // Dispatch ROS trajectory
  auto goal_handle_future = traj_client_->async_send_goal(traj_);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
  }
}

/* ************************************************************************** */
void Traj::publishEstimatedTrajectory(gtsam::Values& values, Problem& problem, size_t step)
{
  gtsam::Vector conf;
  gtsam::Pose2 pose;
  trajectory_msgs::msg::JointTrajectory est_traj;
  est_traj.points.resize(step + 1);
  for (std::size_t i = 0; i < step + 1; i++)
  {
    if (!problem.robot.isMobileBase())
    {
      est_traj.points[i].positions.resize(problem.robot.getDOFarm());
      conf = values.at<gtsam::Vector>(gtsam::Symbol('x', i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      for (std::size_t j = 0; j < problem.robot.getDOFarm(); j++)
        est_traj.points[i].positions[j] = conf[j];
    }
    else
    {
      est_traj.points[i].positions.resize(problem.robot.getDOF());
      pose = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i)).pose();
      conf = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i)).configuration();
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      est_traj.points[i].positions[0] = pose.x();
      est_traj.points[i].positions[1] = pose.y();
      est_traj.points[i].positions[2] = pose.theta();
      for (std::size_t j = 0; j < problem.robot.getDOFarm(); j++)
        est_traj.points[i].positions[j + 3] = conf[j];
    }
  }
  est_traj_pub->publish(est_traj);
}

/* ************************************************************************** */
void Traj::publishPlannedTrajectory(gtsam::Values& values, Problem& problem, size_t step)
{
  gtsam::Vector conf;
  gtsam::Pose2 pose;
  trajectory_msgs::msg::JointTrajectory plan_traj;
  plan_traj.points.resize(problem.total_step - step);
  for (std::size_t i = step; i < problem.total_step; i++)
  {
    if (!problem.robot.isMobileBase())
    {
      plan_traj.points[i - step].positions.resize(problem.robot.getDOFarm());
      conf = values.at<gtsam::Vector>(gtsam::Symbol('x', i));
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      for (std::size_t j = 0; j < problem.robot.getDOFarm(); j++)
        plan_traj.points[i - step].positions[j] = conf[j];
    }
    else
    {
      plan_traj.points[i - step].positions.resize(problem.robot.getDOF());
      pose = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i)).pose();
      conf = values.at<gpmp2::Pose2Vector>(gtsam::Symbol('x', i)).configuration();
      if (problem.robot.isThetaNeg())
        problem.robot.negateTheta(conf);
      plan_traj.points[i - step].positions[0] = pose.x();
      plan_traj.points[i - step].positions[1] = pose.y();
      plan_traj.points[i - step].positions[2] = pose.theta();
      for (std::size_t j = 0; j < problem.robot.getDOFarm(); j++)
        plan_traj.points[i - step].positions[j + 3] = conf[j];
    }
  }
  plan_traj_pub->publish(plan_traj);
}

} // namespace piper
