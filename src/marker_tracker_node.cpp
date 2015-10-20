/****************************************************************************
 *
 *   Copyright (c) 2015 Crossline Drone Project Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name CLDrone nor Crossline Drone nor the names of its c
 *    ontributors may be used to endorse or promote products derived from 
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>

ros::Publisher bodyAxisPositionPublisher;

void markerPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
   geometry_msgs::PoseStamped markerPose = *msg;
   geometry_msgs::PoseStamped ps;

   // Front camera tracking
   //ps.pose.position.x = markerPose.pose.position.z - 0.5;
   //ps.pose.position.z = -markerPose.pose.position.y;
   //ps.pose.position.y = -markerPose.pose.position.x;
   //ps.pose.orientation.w = 1;

   // Downward camera tracking
   ps.pose.position.x = -markerPose.pose.position.y;
   ps.pose.position.y = -markerPose.pose.position.x;
   ps.pose.position.z = -(markerPose.pose.position.z - 0.5);

   tf::Pose currentPose;
   tf::poseMsgToTF(markerPose.pose,currentPose);
   double yawAngle = tf::getYaw(currentPose.getRotation());
   ROS_INFO_STREAM("yawAngle:" << yawAngle);
   ps.pose.orientation = tf::createQuaternionMsgFromYaw(-yawAngle);

   ps.header.stamp = ros::Time::now();

   bodyAxisPositionPublisher.publish(ps);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_tracker_node");
  ros::NodeHandle nodeHandle;

  bodyAxisPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/CLDrone/body_axis_position/local",10);
  ros::Subscriber markerPoseSubscriber = nodeHandle.subscribe("/aruco_single/pose",10,markerPoseReceived);
  

  ros::spin();

  return 0;

}