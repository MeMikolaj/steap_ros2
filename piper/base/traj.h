/**
 *  @file   traj.h
 *  @brief  trajectory: action client, initialize, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 * 
 *  @details ROS2 modifications made by by Mikolaj Kliniewski Oct 3, 2024
 **/

#ifndef TRAJ_H_
#define TRAJ_H_

#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gpmp2/geometry/Pose2Vector.h>

#include <problem.h>
#include <misc.h>

namespace piper {

class Traj
{
public:
    using TrajClient = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr est_traj_pub, 
        plan_traj_pub;
    std::vector<std::string> arm_joint_names;

private:
    std::string trajectory_control_topic_, est_traj_pub_topic_, plan_traj_pub_topic_;
    std::shared_ptr<TrajClient> traj_client_;
    control_msgs::action::FollowJointTrajectory::Goal traj_;

public:
    /// Default constructor
    Traj() {}

    /**
     *  Setup traj client and load params
     *
     *  @param node shared pointer to the rclcpp node
     **/
    Traj(rclcpp::Node::SharedPtr node);

    /// Default destructor
    virtual ~Traj() {}

    /**
     *  Initialize trajectory for optimization
     *
     *  @param init_values initialized traj saved in this variable
     *  @param problem all problem params and settings
     **/
    void initializeTrajectory(gtsam::Values& init_values, Problem& problem);

    /**
     *  Execute trajectory
     *
     *  @param exec_values optimized, interpolated, and collision-checked traj to execute
     *  @param problem all problem params and settings
     *  @param exec_step number of points on the trajectory
     **/
    void executeTrajectory(gtsam::Values& exec_values, Problem& problem, size_t exec_step);

    /**
     *  Publish estimated trajectory
     *
     *  @param values estimated part of traj
     *  @param problem all problem params and settings
     *  @param step estimated traj is from 0 to step
     **/
    void publishEstimatedTrajectory(gtsam::Values& values, Problem& problem, size_t step);

    /**
     *  Publish planned trajectory
     *
     *  @param values planned part of traj
     *  @param problem all problem params and settings
     *  @param step planned traj is from step to total_step
     **/
    void publishPlannedTrajectory(gtsam::Values& values, Problem& problem, size_t step);
};

} // namespace piper

#endif
