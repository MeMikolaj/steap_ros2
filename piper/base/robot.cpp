/**
 *  @file   robot.h
 *  @brief  robot: load, create, etc
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 * 
 *  @details ROS2 modifications made by by Mikolaj Kliniewski Oct 3, 2024
 **/

#include "robot.h"

namespace piper {

/* ************************************************************************** */
Robot::Robot(rclcpp::Node::SharedPtr node)
{
    // get parameters for desired robot
    RCLCPP_INFO(node->get_logger(), "Loading robot parameters.");
    
    // Use the Node to get parameters
    node->get_parameter("robot/DOF", DOF_);
    node->get_parameter("robot/mobile_base", mobile_base_);
    if (!node->get_parameter("robot/arm_base/orientation", orientation_))
        orientation_ = {0, 0, 0, 1}; // Default quaternion

    if (!node->get_parameter("robot/arm_base/position", position_))
        position_ = {0, 0, 0}; // Default position

    node->get_parameter("robot/DH/a", a_);
    node->get_parameter("robot/DH/alpha", alpha_);
    node->get_parameter("robot/DH/d", d_);
    node->get_parameter("robot/DH/theta", theta_);
    node->get_parameter("robot/DH/theta_neg", theta_neg_);

    node->get_parameter("robot/spheres/js", js_);
    node->get_parameter("robot/spheres/xs", xs_);
    node->get_parameter("robot/spheres/ys", ys_);
    node->get_parameter("robot/spheres/zs", zs_);
    node->get_parameter("robot/spheres/rs", rs_);
    
    node->get_parameter("robot/sensor_arm_sigma", sensor_arm_sigma);
    if (sensor_arm_sigma == 0.0) sensor_arm_sigma = 0.0001; // Default value

    node->get_parameter("robot/sensor_base_sigma", sensor_base_sigma);
    if (sensor_base_sigma == 0.0) sensor_base_sigma = 0.0001; // Default value

    // arm's base pose (relative to robot base if mobile_base_ is true)
    arm_base_ = gtsam::Pose3(gtsam::Rot3::Quaternion(orientation_[3], orientation_[0], orientation_[1], 
        orientation_[2]), gtsam::Point3(getVector(position_)));
    
    // spheres to approximate robot body: js - link id, rs - radius, [xs, ys, zs] - center
    for (size_t i = 0; i < js_.size(); i++)
        spheres_data_.emplace_back(js_[i], rs_[i], gtsam::Point3(xs_[i], ys_[i], zs_[i]));
    
    // generate arm/mobile arm
    if (!mobile_base_)
    {
        DOF_arm_ = DOF_;
        arm = gpmp2::ArmModel(gpmp2::Arm(DOF_arm_, getVector(a_), getVector(alpha_), getVector(d_), 
            arm_base_, getVector(theta_)), spheres_data_);
    }
    else
    {
        DOF_arm_ = DOF_ - 3;
        marm = gpmp2::Pose2MobileArmModel(gpmp2::Pose2MobileArm(gpmp2::Arm(DOF_arm_, getVector(a_), 
            getVector(alpha_), getVector(d_), gtsam::Pose3(), getVector(theta_)), arm_base_), 
            spheres_data_);
    }
}

/* ************************************************************************** */
void Robot::negateTheta(gtsam::Vector& conf)
{
    for (std::size_t i = 0; i < conf.size(); i++)
        if (theta_neg_[i])
            conf[i] *= -1.0;
}

} // namespace piper
