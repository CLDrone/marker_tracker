#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

ros::Publisher bodyAxisPositionPublisher;

void markerPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
   geometry_msgs::PoseStamped markerPose = *msg;
   geometry_msgs::PoseStamped ps;
   ps.pose.position.x = markerPose.pose.position.z - 0.5;
   ps.pose.position.z = -markerPose.pose.position.y;
   ps.pose.position.y = -markerPose.pose.position.x;

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