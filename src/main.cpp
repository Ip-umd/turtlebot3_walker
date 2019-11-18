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
 * @file main.cpp
 *
 * @date 17 November 2019
 *
 * @author Ishan Patel
 *
 * @brief Main program to run turtlebot3
 *        using walker algorithm
 * @version 1
 *
 */
#include "walker.hpp"

/**
 * @brief main function
 * @param argc count of arguments passed on command line
 * @param argv Stores all commandline arguments
 * @return 0 for successful execution
 *         
 */
int main(int argc, char **argv) {
  /// Initialize the ros node
  ros::init(argc, argv, "walker");
  /// Creating an object of Walker class
  Walker turtlebot;
  /// Check if the node is successfully initialized
  if (ros::isInitialized()) {
     turtlebot.moveTurtle();
  } else {
     ROS_FATAL_STREAM("ROS is not initialized.");
  }
  return 0;
}
