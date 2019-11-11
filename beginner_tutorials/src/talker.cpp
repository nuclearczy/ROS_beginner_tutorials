/** Copyright (c) 2019   Zuyang Cao
 *  @file       talker.cpp
 *  @brief      A simple ROS beginner tutorial for publisher and launch param.
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
#include <sstream>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

/** @brief   Main function for the subscriber
 *  @param   ROS argc
 *  @param   ROS argv
 *  @return  0 
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(4);
  int count = 0;
  std::string talkerName;
  std::string messageContent;
  // Setting up tf broadcast and object.
  tf::TransformBroadcaster br;
  tf::Transform transform;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    nh_private.getParam("talker_name", talkerName);
    nh_private.getParam("message_content", messageContent);
    ss << messageContent << " from " << talkerName << count;
    msg.data = ss.str();
    ROS_INFO_STREAM(msg.data);
    chatter_pub.publish(msg);
    // Setting up tf.
    transform.setOrigin(tf::Vector3(1, 2, 3));
    tf::Quaternion qRotation;
    qRotation.setRPY(0.1*count, 2, 3);
    transform.setRotation(qRotation);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
