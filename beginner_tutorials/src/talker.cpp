/** Copyright (c) 2019   Zuyang Cao
 *  @file       talker.cpp
 *  @brief      A simple ROS beginner tutorial for publisher and launch param.
 *  @license    This project is released under the BSD-3-Clause License.
 */
#include <sstream>
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
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    nh_private.getParam("talker_name", talkerName);
    nh_private.getParam("message_content", messageContent);
    ss << messageContent << " from " << talkerName << count;
    msg.data = ss.str();
    ROS_INFO_STREAM(msg.data);
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
