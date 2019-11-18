/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Ishan Patel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * @file walker.hpp
 *
 * @date 17 November 2019
 *
 * @author Ishan Patel
 *
 * @brief Declaration of walker class.
 *
 * @version 1
 *
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

class Walker {
 private:
  /// Creating a ROS node handle
  ros::NodeHandle n;
  /// Creating ROS publisher
  ros::Publisher pub;
  /// Creating ROS subscriber
  ros::Subscriber sub;
  /// Variable used for publishing velocity
  geometry_msgs::Twist pos;
  /// Variable for detecting collision
  bool checkCollision;

 public:
  /**
    * @brief Constructor for walker class
    */
  Walker();
  /**
    * @brief Destructor for walker class
    */
  ~Walker();
  /**
    * @brief A callback function called upon arrival of a 
    *	   new message at /scan topic
    * @param data A boost shared pointer that points to 
    *	   messages on /scan topic
    * @return void
    */
  void checkObstacle(const sensor_msgs::LaserScan::ConstPtr& data);
   /**
     * @brief The main function of the walker algorithm which
     *        publishes velocity messages according
     *        to the information from laser scan.
     * @return void
     */
  void moveTurtle();
};
#endif  // INCLUDE_WALKER_HPP_

