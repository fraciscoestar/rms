#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <rms/TakeOff.h>
#include <rms/Land.h>
#include <rms/GoToWaypoint.h>

enum DroneState { Landed, TakingOff, Idle, Moving, Landing };
DroneState droneState = Landed;
bool blocked = false;

std::string robotName = "quadrotor";
geometry_msgs::Pose currentPose;
geometry_msgs::Pose targetPose;
geometry_msgs::Twist currentVelocity;
geometry_msgs::Twist targetVelocity;

double frequency = 120.;

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

bool TakeOffServiceCalled(rms::TakeOff::Request& req, rms::TakeOff::Response& res)
{
    if (droneState == Landed)
    {
        targetPose = currentPose;
        targetPose.position.z = req.height;        
        droneState = TakingOff;
        blocked = true;

        res.ok = true;
        return true;
    }
    else
    {
        res.ok = false;
        return false;
    }
}

bool LandServiceCalled(rms::Land::Request& req, rms::Land::Response& res)
{
    if (droneState == Idle || (droneState == Moving && !blocked))
    {
        targetPose = currentPose;
        targetPose.position.z = 0;        
        droneState = Landing;
        blocked = true;

        res.ok = true;
        return true;
    }
    else
    {
        res.ok = false;
        return false;
    }
}

bool GoToWaypointServiceCalled(rms::GoToWaypoint::Request& req, rms::GoToWaypoint::Response& res)
{
    if(droneState == Idle || (droneState == Moving && !blocked))
    {
        targetPose.position = req.waypoint;
        blocked = req.blocking;
        droneState = Moving;

        res.ok = true;
        return true;
    }
    else
    {
        res.ok = false;
        return false;
    }
}

geometry_msgs::Twist GetVelocityRef(geometry_msgs::Point targetPoint)
{
    double max_horizontal_velocity = 10.0;
    double max_vertical_velocity = 10.0;
    double max_position_error = 0.01;
    geometry_msgs::Twist vel;

    double dx = targetPoint.x - currentPose.position.x;
    double dy = targetPoint.y - currentPose.position.y;
    double dz = targetPoint.z - currentPose.position.z;

    double Th = sqrt( dx*dx + dy*dy ) / max_horizontal_velocity;
    double Tz = std::abs( dz / max_vertical_velocity);
    double T = std::max(Th, Tz);

    if ( T < 1/frequency ) 
        T = 1/frequency;

    vel.linear.x = dx / T;
    vel.linear.y = dy / T;
    vel.linear.z = dz / T;

    if (std::abs(dx) < max_position_error)
        vel.linear.x = 0;
    if (std::abs(dy) < max_position_error)
        vel.linear.y = 0;
    if (std::abs(dz) < max_position_error)
        vel.linear.z = 0;

    if (std::abs(dx) < max_position_error && std::abs(dy) < max_position_error && std::abs(dz) < max_position_error)
    {
        switch (droneState)
        {
        case Landed:
            break;

        case TakingOff:
            droneState = Idle;
            break;

        case Idle:
            break;

        case Moving:
            droneState = Idle;
            break;

        case Landing:
            droneState = Landed;
            break;
        
        default:
            break;
        }

        blocked = false;
    }

    return vel;
}

geometry_msgs::Pose Move()
{
    double dt = 1./frequency;

    currentVelocity.linear.x = (0.4 * targetVelocity.linear.x + 0.6 * currentVelocity.linear.x);
    currentVelocity.linear.y = (0.4 * targetVelocity.linear.y + 0.6 * currentVelocity.linear.y);
    currentVelocity.linear.z = (0.4 * targetVelocity.linear.z + 0.6 * currentVelocity.linear.z);

    geometry_msgs::Pose pose;

    if (targetVelocity.linear.x != 0)
        pose.position.x = currentPose.position.x + dt * currentVelocity.linear.x;
    else
        pose.position.x = targetPose.position.x;

    if (targetVelocity.linear.y != 0)
        pose.position.y = currentPose.position.y + dt * currentVelocity.linear.y;
    else
        pose.position.y = targetPose.position.y;

    if (targetVelocity.linear.z != 0)
        pose.position.z = currentPose.position.z + dt * currentVelocity.linear.z;
    else
        pose.position.z = targetPose.position.z;

    return pose;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "drone_control");

    ros::service::waitForService("/gazebo/spawn_sdf_model");

    ros::NodeHandle nh;
    ros::Subscriber gazeboSubscriber = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, GazeboSubscriber);

    ros::Publisher publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

    ros::ServiceServer takeOffService = nh.advertiseService("/take_off", TakeOffServiceCalled);
    ros::ServiceServer landService = nh.advertiseService("/land", LandServiceCalled);
    ros::ServiceServer goToWaypointService = nh.advertiseService("/go_to_waypoint", GoToWaypointServiceCalled);

    ros::Rate rate(frequency);
    while(ros::ok())
    {
        targetVelocity = GetVelocityRef(targetPose.position);

        gazebo_msgs::ModelState msg;
        msg.model_name = robotName;
        msg.pose = Move();

        publisher.publish(msg);
        
        ros::spinOnce();
        rate.sleep();
    }
}