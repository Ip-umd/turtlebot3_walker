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
 * @file walker.cpp
 *
 * @date 17 November 2019
 *
 * @author Ishan Patel
 *
 * @brief Implement the methods
 *        of walker class
 * @version 1
 *
 */

#include "walker.hpp"

/**
 * @brief  Constructs the object of the class Walker.
 */
Walker::Walker() {
  /// Initialising collision check variable
  checkCollision = false;
}
/**
 * @brief  Destroys the object of class walker.
 */
Walker::~Walker() {
        /// Stopping the turtlebot at the end of the program
        pos.linear.x = 0.0;
        pos.linear.y = 0.0;
        pos.linear.z = 0.0;
        pos.angular.x = 0.0;
        pos.angular.y = 0.0;
        pos.angular.z = 0.0;
        pub.publish(pos);
}

/**
 * @brief  Callback function for the laser scan data
 * @param  data Message from /scan topic
 */
void Walker::checkObstacle(const sensor_msgs::LaserScan::ConstPtr& data) {
        for (auto d : data->ranges) {
                if (d < 1.0) {
                        checkCollision = true;
                        break;
                }
                checkCollision = false;
        }
}

/**
 * @brief  Main function for walker algorithm
 * @return void
 */

void Walker::moveTurtle() {
        /// Publishing velocity messages on /cmd_vel_mux/input/navi topic
        pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi",
                                               1000);
        /// Subscribing to topic /scan and using checkObstacle method
        sub = n.subscribe <sensor_msgs::LaserScan> ("/scan", 50,
                                                  &Walker::checkObstacle, this);
        /// Setting publishing rate
        ros::Rate loop_rate(10);
        while (ros::ok()) {
                if (checkCollision) {
                        ROS_INFO_STREAM("Obstacle Detected!");
                        /// Stopping the robot and turning it
                        pos.linear.x = 0.0;
                        pos.angular.z = -0.5;
                } else {
                        ROS_INFO_STREAM("Moving Straight.");
                        /// Stopped turning and moving straight
                        pos.linear.x = 0.3;
                        pos.angular.z = 0.0;
                }
                /// Publishing the velosity
                pub.publish(pos);
                ros::spinOnce();
                loop_rate.sleep();
        }
}

