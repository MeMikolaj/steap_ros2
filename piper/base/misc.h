/**
 *  @file   misc.h
 *  @brief  miscellaneous functions
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 * 
 *  @details ROS2 modifications made by by Mikolaj Kliniewski Oct 3, 2024
 **/

#ifndef MISC_H_
#define MISC_H_

#include <csignal>
#include <vector>
#include <stdint.h>
#include <string>

#include <rclcpp/rclcpp.hpp> // Include for ROS 2
#include <rclcpp/node.hpp>
#include <gtsam/base/Vector.h>

namespace piper {

/**
 *  Convert std double vector type to gtsam Vector
 *
 *  @param v std double vector
 **/
static const gtsam::Vector getVector(const std::vector<double>& v)
{
  gtsam::Vector send(v.size());
  for (std::size_t i = 0u; i < v.size(); i++)
    send[i] = v[i];
  return send;
}

/**
 *  CTRL+C handling
 **/
static void sigintHandler(int sig)
{
  rclcpp::shutdown(); // Cleanly shutdown ROS 2
  exit(EXIT_FAILURE); // Terminate the program
}

} // namespace piper

#endif // MISC_H_
