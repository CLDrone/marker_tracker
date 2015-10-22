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
geometry_msgs::PoseStamped ps,lastMarkerPose;

bool hasLastMarkerPose;
ros::Time lastTime;

void markerPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
   geometry_msgs::PoseStamped markerPose = *msg;
   
   // Front camera tracking
   ps.pose.position.x = (markerPose.pose.position.z - 2.5)/2;
   ps.pose.position.z = -markerPose.pose.position.y/2;
   ps.pose.position.y = -markerPose.pose.position.x/2;
   ros::Time currentTime = ros::Time::now();

   if(hasLastMarkerPose){
      geometry_msgs::Pose offsetPose;
      offsetPose.position.x = markerPose.pose.position.x - lastMarkerPose.pose.position.x;
      offsetPose.position.y = markerPose.pose.position.y - lastMarkerPose.pose.position.y;
      offsetPose.position.z = markerPose.pose.position.z - lastMarkerPose.pose.position.z;
      ros::Duration offsetTime = currentTime - lastTime;
      offsetPose.position.x = offsetPose.position.x / offsetTime.toSec();
      offsetPose.position.y = offsetPose.position.y / offsetTime.toSec();
      offsetPose.position.z = offsetPose.position.z / offsetTime.toSec();

      //ROS_INFO_STREAM("offset :" << offsetPose.position.x << offsetPose.position.y << offsetPose.position.z);
      ps.pose.position.x = ps.pose.position.x - offsetPose.position.z/2;
      ps.pose.position.z = ps.pose.position.z - offsetPose.position.y/2;
      ps.pose.position.y = ps.pose.position.y - offsetPose.position.x/2;
   
   }

    if (ps.pose.position.x > 0.5)
    {
      ps.pose.position.x = 0.5;
    }
    if (ps.pose.position.y > 0.5)
    {
      ps.pose.position.y = 0.5;
    }
    if (ps.pose.position.z > 0.5)
    {
      ps.pose.position.z = 0.5;
    }
    if (ps.pose.position.x < -0.5)
    {
      ps.pose.position.x = -0.5;
    }
    if (ps.pose.position.y < -0.5)
    {
      ps.pose.position.y = -0.5;
    }
    if (ps.pose.position.z < -0.5)
    {
      ps.pose.position.z = -0.5;
    }
    

   

   // Downward camera tracking
   //ps.pose.position.x = -markerPose.pose.position.y;
   //ps.pose.position.y = -markerPose.pose.position.x;
   //ps.pose.position.z = -(markerPose.pose.position.z - 0.5);
   
   /*
   tf::Pose currentPose;
   tf::poseMsgToTF(markerPose.pose,currentPose);
   double yawAngle = tf::getYaw(currentPose.getRotation());
   ROS_INFO_STREAM("yawAngle:" << yawAngle);
   ps.pose.orientation = tf::createQuaternionMsgFromYaw(-yawAngle);
   */
   
   lastMarkerPose = markerPose;
   lastTime = currentTime;
   hasLastMarkerPose = true;
   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_tracker_node");
  ros::NodeHandle nodeHandle;

  bodyAxisPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/CLDrone/body_axis_position/local",10);
  ros::Subscriber markerPoseSubscriber = nodeHandle.subscribe("/aruco_single/pose",10,markerPoseReceived);

  ps.pose.orientation.w = 1;

  hasLastMarkerPose = false;

  ros::Rate loopRate(10.0);
  while(ros::ok())
  {
    ps.header.seq++;
	  ps.header.stamp = ros::Time::now();

	  
	  bodyAxisPositionPublisher.publish(ps);
	  
    
  	ros::spinOnce();
  	loopRate.sleep();
  }

  return 0;

}