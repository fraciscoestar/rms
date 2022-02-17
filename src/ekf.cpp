#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <rms/BeaconMsg.h>

std::vector<double> measurements; // Format: [xgps, ygps, zgps, zh, d1, d2, d3, ..., dn]' need to preallocate space
std::vector<geometry_msgs::Point> beaconPoints;

void GPSCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    measurements[0] = msg->x;
    measurements[1] = msg->y;
    measurements[2] = msg->z;
}

void HeightCallback(const std_msgs::Float64::ConstPtr& msg)
{
    measurements[3] = msg->data;
}

void BeaconCallback(const rms::BeaconMsg::ConstPtr& msg)
{
    measurements[3 + msg->id] = msg->dist;
    beaconPoints[msg->id - 1] = msg->point;  
}

int main(int argc, char* argv[])
{
    measurements.reserve(20); // Reserve 20 spaces -> Nmax=16
    beaconPoints.reserve(16); // N spaces for N beacons

    ros::init(argc, argv, "extended_kalman_filter");
    ros::NodeHandle nh;

    ros::service::waitForService("/gazebo/spawn_sdf_model");

    ros::Subscriber gpsSubscriber = nh.subscribe<geometry_msgs::Point>("/gps", 10, GPSCallback);
    ros::Subscriber gpsSubscriber = nh.subscribe<std_msgs::Float64>("/height", 10, HeightCallback);
    ros::Subscriber beaconSubscriber = nh.subscribe<rms::BeaconMsg>("/beacon", 10, BeaconCallback);
}