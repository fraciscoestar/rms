#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include "../include/rms/noiseGenerator.hpp"
#include "rms/BeaconMsg.h"

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

std::string robotName = "quadrotor";
ros::Publisher distancePublisher;

int id = 1;
Vector3 beaconPos = Vector3(0., 0., 0.);
NoiseGenerator noiseGenerator(0.0, 0.0);

void SubscriberCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == robotName)
        {
            Vector3 pose = Vector3(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z);
            double dist = (pose - beaconPos).GetNorm();
            dist = noiseGenerator.AddAWGN(dist);

            rms::BeaconMsg msg;
            msg.id = id;
            msg.x = beaconPos.x;
            msg.y = beaconPos.y;
            msg.z = beaconPos.z;
            msg.dist = dist;
            distancePublisher.publish(msg);
        }
    }
}

int main(int argc, char* argv[])
{
    double mean = 0., stddev = 0.;

    // Argument parsing
    for (int i = 1; i < argc; i += 2)
    {
        if ((std::string)argv[i] == "-id")
        {
            try
            {
                id = atoi(argv[i + 1]);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }                 
        }
        else if ((std::string)argv[i] == "-model")
        {
            robotName = std::string(argv[i + 1]);           
        }
        else if ((std::string)argv[i] == "-x")
        {
            try
            {
                beaconPos.x = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }           
        }
        else if ((std::string)argv[i] == "-y")
        {
            try
            {
                beaconPos.y = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }          
        }
        else if ((std::string)argv[i] == "-z")
        {
            try
            {
                beaconPos.z = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }           
        }
        else if ((std::string)argv[i] == "-mean")
        {
            try
            {
                mean = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }           
        }
        else if ((std::string)argv[i] == "-stddev")
        {
            try
            {
                stddev = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }           
        }       
    }

    ROS_INFO("Creating beacon...");
    ROS_INFO("id     = %d", id);
    ROS_INFO("model  = %s", robotName.c_str());
    ROS_INFO("x      = %f", beaconPos.x);
    ROS_INFO("y      = %f", beaconPos.y);
    ROS_INFO("z      = %f", beaconPos.z);
    ROS_INFO("mean   = %f", mean);
    ROS_INFO("stddev = %f", stddev);
    
    noiseGenerator.Initialize(mean, stddev);

    ros::init(argc, argv, "radio_beacon_" + std::to_string(id));
    ros::NodeHandle nh;

    ros::service::waitForService("/gazebo/spawn_sdf_model");

    ros::Subscriber gazeboSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, SubscriberCallback);
    distancePublisher = nh.advertise<rms::BeaconMsg>("/beacon", 10);

    ros::spin();
}
