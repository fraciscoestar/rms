#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <rms/BeaconMsg.h>
#include <visualization_msgs/Marker.h>

std::string robotName = "quadrotor";
ros::Publisher posePublisher;
ros::Publisher ekfPublisher;
ros::Publisher gpsPublisher;
ros::Publisher beaconPublisher;

void EKFCallback(const gazebo_msgs::ModelState::ConstPtr& msg)
{
    geometry_msgs::PoseStamped ps;
    ps.pose = msg->pose;
    ps.header.frame_id = "map";
    ekfPublisher.publish(ps);
}

void GPSCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    geometry_msgs::PointStamped ps;
    ps.point = *msg;
    ps.header.frame_id = "map";
    gpsPublisher.publish(ps);
}

void BeaconCallback(const rms::BeaconMsg::ConstPtr& msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "beacon_" + std::to_string(msg->id);
    marker.id = msg->id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = msg->point.x;
    marker.pose.position.y = msg->point.y;
    marker.pose.position.z = msg->point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2.0 * msg->dist;
    marker.scale.y = 2.0 * msg->dist;
    marker.scale.z = 2.0 * msg->dist;
  
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.2f;
    marker.color.g = 1.0f;
    marker.color.b = 0.5f;
    marker.color.a = 0.15f;
  
    marker.lifetime = ros::Duration();

    beaconPublisher.publish(marker);
}

void GazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == robotName)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = msg->pose[i].position.x;
            pose.pose.position.y = msg->pose[i].position.y;
            pose.pose.position.z = msg->pose[i].position.z;
            pose.header.frame_id = "map";

            posePublisher.publish(pose);
        }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rviz_helper");
    ros::NodeHandle nh;

    ros::Subscriber ekfSubscriber = nh.subscribe<gazebo_msgs::ModelState>("/ekf", 10, EKFCallback);
    ros::Subscriber poseSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, GazeboCallback);
    ros::Subscriber gpsSubscriber = nh.subscribe<geometry_msgs::Point>("/gps", 10, GPSCallback);
    ros::Subscriber beaconSubscriber = nh.subscribe<rms::BeaconMsg>("/beacon", 10, BeaconCallback);

    posePublisher = nh.advertise<geometry_msgs::PoseStamped>("/drone_pose", 10);
    ekfPublisher = nh.advertise<geometry_msgs::PoseStamped>("/drone_ekf", 10);
    gpsPublisher = nh.advertise<geometry_msgs::PointStamped>("/drone_gps", 10);
    beaconPublisher = nh.advertise<visualization_msgs::Marker>("/drone_beacon", 10);

    ros::spin();
}