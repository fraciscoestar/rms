#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <rms/TakeOff.h>
#include <rms/GoToWaypoint.h>
#include "../include/rms/dronecontrol.hpp"

bool landed = true;
bool busy = false;

std::string robotName = "quadrotor";
geometry_msgs::Pose currentPose;
geometry_msgs::Twist currentVelocity;

double frequency = 60.;

void GazeboSubscriber(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    ros::Rate rate(frequency);
    
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == robotName)
        {
            currentPose = msg->pose[i];
            currentVelocity = msg->twist[i];
        }
    }
}

bool DroneControl::TakeOff(double height)
{
    if (landed && !busy)
    {
        targetPose = currentPose;
        targetPose.position.z = height;
        return true;
    }
    else
    {
        return false;
    }
}

bool DroneControl::GoToWaypoint(const geometry_msgs::Point& waypoint)
{
    if(!busy)
    {
        targetPose.position = waypoint;
        return true;
    }
    else
    {
        return false;
    }
}

geometry_msgs::Twist DroneControl::GetVelocityRef()
{
    geometry_msgs::Pose poseError;
    poseError.position.x = targetPose.position.x - currentPose.position.x;
    poseError.position.y = targetPose.position.y - currentPose.position.y;
    poseError.position.z = targetPose.position.z - currentPose.position.z;
    double distance = sqrt(pow(poseError.position.x, 2) + pow(poseError.position.y, 2) + pow(poseError.position.z, 2));

    //w = w + Kp*vQ + (Ki*Tsampling - Kp)*pVQ; // Forward Euler (PI)
    geometry_msgs::Twist velocity;
    velocity.linear.x = currentVelocity.linear.x + kp*poseError.position.x + (ki*(1./30.) - kp)*lastError;
    velocity.linear.y = currentVelocity.linear.y + kp*poseError.position.y + (ki*(1./30.) - kp)*lastError;
    velocity.linear.z = currentVelocity.linear.z + kp*poseError.position.z + (ki*(1./30.) - kp)*lastError;

    if (velocity.linear.x > 10.)
        velocity.linear.x = 10.;
    if (velocity.linear.y > 10.)
        velocity.linear.y = 10.;
    if (velocity.linear.z > 10.)
        velocity.linear.z = 10.;

    return velocity;
}

geometry_msgs::Pose DroneControl::Move(geometry_msgs::Twist refVelocity)
{
    double dt = 1 / 30.;

    currentVelocity.linear.x = (0.2 * refVelocity.linear.x + 0.8 * currentVelocity.linear.x);
    currentVelocity.linear.y = (0.2 * refVelocity.linear.y + 0.8 * currentVelocity.linear.y);
    currentVelocity.linear.z = (0.2 * refVelocity.linear.z + 0.8 * currentVelocity.linear.z);

    geometry_msgs::Pose pose;

    pose.position.x += dt * currentVelocity.linear.x;
    pose.position.y += dt * currentVelocity.linear.y;
    pose.position.z += dt * currentVelocity.linear.z;

    return pose;
}

DroneControl::DroneControl()
{
    ros::NodeHandle nh;

    ros::ServiceServer takeOffService = nh.advertiseService<rms::TakeOff::Request, rms::TakeOff::Response>("/take_off",
        [this](rms::TakeOff::Request& req, rms::TakeOff::Response& res) 
        { res.ok = true; return TakeOff(req.height); });

    ros::ServiceServer goToWaypointService = nh.advertiseService<rms::GoToWaypoint::Request, rms::GoToWaypoint::Response>("/go_to_waypoint",
        [this](rms::GoToWaypoint::Request& req, rms::GoToWaypoint::Response& res) 
        { res.ok = true; return GoToWaypoint(req.waypoint); });

    ros::Publisher publisher = nh.advertise<gazebo_msgs::ModelStates>("/gazebo/model_states", 10);

    ros::Rate rate(30.);
    while (ros::ok())
    {
        geometry_msgs::Twist velocityRef = GetVelocityRef();
        geometry_msgs::Pose nextPose = Move(velocityRef);

        gazebo_msgs::ModelState msg;
        msg.model_name = robotName;
        msg.pose = nextPose;

        publisher.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "drone_control");

    ros::service::waitForService("/gazebo/spawn_sdf_model");
    DroneControl control = DroneControl();

    ros::NodeHandle nh;
    ros::Subscriber gazeboSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, GazeboSubscriber);

    while(ros::ok())
    {
        control.currentPose = currentPose;
        control.currentVelocity = currentVelocity;

        ros::spinOnce();
    }
}