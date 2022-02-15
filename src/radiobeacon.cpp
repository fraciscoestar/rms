#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include "../include/rms/noiseGenerator.hpp"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>

struct Vector3
{
    double x, y, z;

    Vector3()
    {
        x = 0.;
        y = 0.;
        z = 0.;
    }

    Vector3(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Vector3(const Vector3& vector3)
    {
        x = vector3.x;
        y = vector3.y;
        z = vector3.z;
    }

    Vector3(const Vector3&& vector3)
    {
        x = vector3.x;
        y = vector3.y;
        z = vector3.z;
    }

    Vector3(const geometry_msgs::Pose& pose)
    {
        x = pose.position.x;
        y = pose.position.y;
        z = pose.position.z;
    }

    friend Vector3 operator+(Vector3 v1, const Vector3& v2)
    {
        return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
    }

    friend Vector3 operator-(Vector3 v1, const Vector3& v2)
    {
        return Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
    }

    double GetNorm()
    {
        return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    }
};

const std::string robotName = "quadrotor";
ros::Publisher distancePublisher;

Vector3 beaconPos = Vector3(10., 10., 0.);
NoiseGenerator noiseGenerator(0.0, 5.0);

void SubscriberCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == robotName)
        {
            Vector3 pose = Vector3(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z);
            double dist = (pose - beaconPos).GetNorm();
            dist = noiseGenerator.AddAWGN(dist);

            std_msgs::Float64 msg;
            msg.data = dist;
            distancePublisher.publish(msg);
        }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "radio_beacon");
    ros::NodeHandle nh;

    ros::Subscriber gazeboSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, SubscriberCallback);
    distancePublisher = nh.advertise<std_msgs::Float64>("/beacon", 10);

    ros::spin();
}
