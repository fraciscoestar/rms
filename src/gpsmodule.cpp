#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include "../include/rms/noiseGenerator.hpp"

ros::Publisher heightPublisher;
NoiseGenerator noiseGeneratorX(0., 0.);
NoiseGenerator noiseGeneratorY(0., 0.);
NoiseGenerator noiseGeneratorZ(0., 0.);

std::string robotName = "quadrotor";
double frequency = 10.;

void SubscriberCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    ros::Rate rate(frequency);
    
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == robotName)
        {
            geometry_msgs::Point pose;
            pose.x = noiseGeneratorX.AddAWGN(msg->pose[i].position.x);
            pose.y = noiseGeneratorY.AddAWGN(msg->pose[i].position.y);
            pose.z = std::abs(noiseGeneratorZ.AddAWGN(msg->pose[i].position.z)); // Height cant be negative

            heightPublisher.publish(pose);
            rate.sleep();
        }
    }
}

int main(int argc, char* argv[])
{
    double meanX = 0, stddevX = 0;
    double meanY = 0, stddevY = 0;
    double meanZ = 0, stddevZ = 0;

    // Argument parsing
    for (int i = 1; i < argc; i += 2)
    {
        if ((std::string)argv[i] == "-model")
        {
            robotName = std::string(argv[i + 1]);           
        }
        else if ((std::string)argv[i] == "-meanx")
        {
            try
            {
                meanX = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }           
        }
        else if ((std::string)argv[i] == "-stddevx")
        {
            try
            {
                stddevX = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }           
        }
        else if ((std::string)argv[i] == "-meany")
        {
            try
            {
                meanY = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }           
        }
        else if ((std::string)argv[i] == "-stddevy")
        {
            try
            {
                stddevY = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }           
        }
        else if ((std::string)argv[i] == "-meanz")
        {
            try
            {
                meanZ = atof(argv[i + 1]);              
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }           
        }
        else if ((std::string)argv[i] == "-stddevz")
        {
            try
            {
                stddevZ = atof(argv[i + 1]);              
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

    ROS_INFO("Creating GPS...");
    ROS_INFO("model   = %s", robotName.c_str());
    ROS_INFO("meanX   = %f", meanX);
    ROS_INFO("stddevX = %f", stddevX);
    ROS_INFO("meanY   = %f", meanY);
    ROS_INFO("stddevY = %f", stddevY);
    ROS_INFO("meanZ   = %f", meanZ);
    ROS_INFO("stddevZ = %f", stddevZ);
    ROS_INFO("freq    = %f", frequency);
    
    noiseGeneratorX.Initialize(meanX, stddevX);
    noiseGeneratorY.Initialize(meanY, stddevY);
    noiseGeneratorZ.Initialize(meanZ, stddevZ);

    ros::init(argc, argv, "gps_module");
    ros::NodeHandle nh;
    
    ros::service::waitForService("/gazebo/spawn_sdf_model");

    ros::Subscriber gazeboSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, SubscriberCallback);
    heightPublisher = nh.advertise<geometry_msgs::Point>("/gps", 10);

    ros::spin();
}