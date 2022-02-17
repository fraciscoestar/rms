#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include "../include/rms/noiseGenerator.hpp"
#include "rms/BeaconMsg.h"

std::string robotName = "quadrotor";
ros::Publisher distancePublisher;
double frequency = 10.;

int id = 1;
geometry_msgs::Point beaconPos;
NoiseGenerator noiseGenerator(0.0, 0.0);

void SubscriberCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    ros::Rate rate(frequency);

    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == robotName)
        {
            double dist = sqrt(pow(msg->pose[i].position.x - beaconPos.x, 2) + pow(msg->pose[i].position.y - beaconPos.y, 2) + pow(msg->pose[i].position.z - beaconPos.z, 2));
            dist = abs(noiseGenerator.AddAWGN(dist));

            rms::BeaconMsg msg;
            msg.id = id;
            msg.point = beaconPos;
            msg.dist = dist;
            distancePublisher.publish(msg);
            rate.sleep();
        }
    }
}

int main(int argc, char* argv[])
{
    double mean = 0., stddev = 0.;
    beaconPos.x = 0;
    beaconPos.y = 0;
    beaconPos.z = 0;

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
        else if ((std::string)argv[i] == "-freq")
        {
            try
            {
                frequency = atof(argv[i + 1]);              
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
    ROS_INFO("freq   = %f", frequency);
    
    noiseGenerator.Initialize(mean, stddev);

    ros::init(argc, argv, "radio_beacon_" + std::to_string(id));
    ros::NodeHandle nh;

    ros::service::waitForService("/gazebo/spawn_sdf_model");

    ros::Subscriber gazeboSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, SubscriberCallback);
    distancePublisher = nh.advertise<rms::BeaconMsg>("/beacon", 10);

    ros::spin();
}
