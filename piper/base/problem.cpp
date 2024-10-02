/**
 *  @file   problem.cpp
 *  @brief  problem: load, create, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 *
 *  @details ROS2 modifications made by by Mikolaj Kliniewski Oct 3, 2024
 **/

#include "problem.h"
#include <rclcpp/rclcpp.hpp> // Include for ROS 2 logging and node management
#include <rclcpp/parameter.hpp>

namespace piper {

/* ************************************************************************** */
Problem::Problem(const std::shared_ptr<rclcpp::Node> &node)
{
  // first load robot
  robot = Robot(node);
  
  // given the robot load the planning problem
  RCLCPP_INFO(node->get_logger(), "Loading planning problem.");
  
  // start and goal
  if (node->has_parameter("start_conf"))
  {
    node->get_parameter("start_conf", sc_);
    start_conf = getVector(sc_);
    if (robot.isThetaNeg())
      robot.negateTheta(start_conf);
  }
  
  node->get_parameter("goal_conf", gc_);
  goal_conf = getVector(gc_);
  if (robot.isThetaNeg())
    robot.negateTheta(goal_conf);
  
  if (robot.isMobileBase())
  {
    if (node->has_parameter("start_pose"))
    {
      node->get_parameter("start_pose", sp_);
      start_pose = gtsam::Pose2(sp_[0], sp_[1], sp_[2]);
    }
    node->get_parameter("goal_pose", gp_);
    goal_pose = gtsam::Pose2(gp_[0], gp_[1], gp_[2]);
    pgoal = gpmp2::Pose2Vector(goal_pose, goal_conf);
  }

  // signed distance field
  std::string sdf_file_param;
  node->get_parameter("sdf_file", sdf_file_param);
  sdf_file_ = rclcpp::package::get_package_share_directory("piper") + "/" + sdf_file_param; // Updated path retrieval
  std::string fext = sdf_file_.substr(sdf_file_.find_last_of(".") + 1);
  if (fext == "vol")
    gpmp2::readSDFvolfile(sdf_file_, sdf);
  else
    sdf.loadSDF(sdf_file_);
  
  // optimization settings
  node->get_parameter("total_time", total_time);
  node->get_parameter("total_step", total_step);
  node->get_parameter("obs_check_inter", obs_check_inter);
  node->get_parameter("control_inter", control_inter);
  node->get_parameter("cost_sigma", cost_sigma);
  node->get_parameter("epsilon", epsilon);
  node->get_parameter("opt_type", opt_type_);
  node->get_parameter("Qc", Qc_);
  node->get_parameter("fix_pose_sigma", fix_pose_sigma_);
  node->get_parameter("fix_vel_sigma", fix_vel_sigma_);
  
  int DOF = robot.getDOF();
  opt_setting = gpmp2::TrajOptimizerSetting(DOF);
  opt_setting.total_time = total_time;
  opt_setting.total_step = total_step - 1;
  delta_t = total_time / (total_step - 1);
  opt_setting.obs_check_inter = obs_check_inter;
  opt_setting.cost_sigma = cost_sigma;
  opt_setting.epsilon = epsilon;
  opt_setting.Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc_ * gtsam::Matrix::Identity(DOF, DOF));
  opt_setting.conf_prior_model = gtsam::noiseModel::Isotropic::Sigma(DOF, fix_pose_sigma_);
  opt_setting.vel_prior_model = gtsam::noiseModel::Isotropic::Sigma(DOF, fix_vel_sigma_);
  
  if (opt_type_ == "LM")
    opt_setting.opt_type = gpmp2::TrajOptimizerSetting::LM;
  else if (opt_type_ == "Dogleg")
    opt_setting.opt_type = gpmp2::TrajOptimizerSetting::Dogleg;
  else if (opt_type_ == "GaussNewton")
    opt_setting.opt_type = gpmp2::TrajOptimizerSetting::GaussNewton;
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Optimization type '%s' not known!", opt_type_.c_str());
    // Shutdown ROS 2 and terminate the program
    RCLCPP_FATAL(node->get_logger("piper"), "Quitting...");
    sigintHandler(0)
  }
}

} // namespace piper
