#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include "../include/rms/noiseGenerator.hpp"

ros::Publisher gpsPublisher;
NoiseGenerator noiseGenerator(0., 0.);

std::string robotName = "quadrotor";

void SubscriberCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == robotName)
        {
            geometry_msgs::Point pose;
            pose.x = noiseGenerator.AddAWGN(msg->pose[i].position.x);
            pose.y = noiseGenerator.AddAWGN(msg->pose[i].position.y);
            pose.z = noiseGenerator.AddAWGN(msg->pose[i].position.z);

            gpsPublisher.publish(pose);
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
    }

    ROS_INFO("Creating GPS...");
    ROS_INFO("model  = %s", robotName.c_str());
    ROS_INFO("mean   = %f", mean);
    ROS_INFO("stddev = %f", stddev);
    
    noiseGenerator.Initialize(mean, stddev);

    ros::init(argc, argv, "gps_module");
    ros::NodeHandle nh;
    
    ros::service::waitForService("/gazebo/spawn_sdf_model");

    ros::Subscriber gazeboSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, SubscriberCallback);
    gpsPublisher = nh.advertise<geometry_msgs::Point>("/gps", 10);

    ros::spin();
}