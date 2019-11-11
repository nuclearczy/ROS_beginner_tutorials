/** Copyright (c) 2019   Zuyang Cao
 *  @file       listener.cpp
 *  @brief      A simple ROS beginner tutorial for subscriber and service.
 *  @license    BSD 3-Clause LICENSE
 *
 * Copyright (c) 2018, Zuyang Cao
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without  
 * modification, are permitted provided that the following conditions are 
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the   
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <std_srvs/Empty.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

/** @brief Indicates the status of this node. */
bool listenStatus = true;

/** @brief   Callback function for service which switch the listener's status
 *  @param   Service request type variable
 *  @param   Service response type variable
 *  @return  boolean true 
 */
// Const will fail the build, but this will cause cpplint error.
bool toggleListenStatus(std_srvs::Empty::Request &req,
std_srvs::Empty::Response &resp) {
  ROS_DEBUG_STREAM("Starting to change status.");
  listenStatus = !listenStatus;
  ROS_WARN_STREAM("Now entering " << (listenStatus ? "Listening" : "Deaf")
  << " status." << std::endl);
  if (!listenStatus) {
    ROS_ERROR_STREAM("HELP!!");
    ROS_FATAL_STREAM("I AM DEAF!!");
  }
  return true;
}


/** @brief   Callback function for subscriber
 *  @param   Msg type message
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  if (listenStatus) {
    ROS_INFO_STREAM("I heard: " << msg->data);
  }
}

/** @brief   Main function for the subscriber
 *  @param   ROS argc
 *  @param   ROS argv
 *  @return  0 
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("toggle_listen_status",
  &toggleListenStatus);
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
