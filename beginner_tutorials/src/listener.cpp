/** Copyright (c) 2019   Zuyang Cao
 *  @file       listener.cpp
 *  @brief      A simple ROS beginner tutorial for subscriber and service.
 *  @license    This project is released under the BSD-3-Clause License.
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
bool toggleListenStatus (std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
  ROS_DEBUG_STREAM("Starting to change status. ");
  listenStatus = !listenStatus ;
  ROS_WARN_STREAM("Now entering " << (listenStatus ? "Listening" : "Deaf") << " status." << std::endl);
  if (!listenStatus){
    ROS_ERROR_STREAM("HELP!!");
    ROS_FATAL_STREAM("I AM DEAF!!");
  }
  return true;
}


/** @brief   Callback function for subscriber
 *  @param   Msg type message
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  if (listenStatus){
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

  ros::ServiceServer server = nh.advertiseService("toggle_listen_status", &toggleListenStatus);

  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
