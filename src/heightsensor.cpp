#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float64.h>
#include "../include/rms/noiseGenerator.hpp"

ros::Publisher heightPublisher;
NoiseGenerator noiseGenerator(0., 0.);

std::string robotName = "quadrotor";
double frequency = 10.;

void SubscriberCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    ros::Rate rate(frequency);

    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == robotName)
        {
            std_msgs::Float64 height;
            height.data = abs(noiseGenerator.AddAWGN(msg->pose[i].position.z)); // Negative measurements have no physical meaning
            heightPublisher.publish(height);
            rate.sleep();
        }
    }
}

int main(int argc, char* argv[])
{
    double mean = 0, stddev = 0;

    // Argument parsing
    for (int i = 1; i < argc; i += 2)
    {
        if ((std::string)argv[i] == "-model")
        {
            robotName = std::string(argv[i + 1]);           
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

    ROS_INFO("Creating height sensor...");
    ROS_INFO("model  = %s", robotName.c_str());
    ROS_INFO("mean   = %f", mean);
    ROS_INFO("stddev = %f", stddev);
    ROS_INFO("freq   = %f", frequency);
    
    noiseGenerator.Initialize(mean, stddev);

    ros::init(argc, argv, "height_sensor");
    ros::NodeHandle nh;
    
    ros::service::waitForService("/gazebo/spawn_sdf_model");

    ros::Subscriber gazeboSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, SubscriberCallback);
    heightPublisher = nh.advertise<std_msgs::Float64>("/height", 10);

    ros::spin();
}