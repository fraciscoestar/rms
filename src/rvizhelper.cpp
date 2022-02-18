#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>

std::string robotName = "quadrotor";
ros::Publisher posePublisher;
ros::Publisher ekfPublisher;
ros::Publisher gpsPublisher;

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

    posePublisher = nh.advertise<geometry_msgs::PoseStamped>("/drone_pose", 10);
    ekfPublisher = nh.advertise<geometry_msgs::PoseStamped>("/drone_ekf", 10);
    gpsPublisher = nh.advertise<geometry_msgs::PointStamped>("drone_gps", 10);

    ros::spin();
}