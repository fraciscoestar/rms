#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>

class DroneControl
{
public:
    bool landed = true;
    bool busy = false;

    geometry_msgs::Pose currentPose;
    geometry_msgs::Twist currentVelocity;
    geometry_msgs::Pose targetPose;

    bool TakeOff(double height);
    bool GoToWaypoint(const geometry_msgs::Point& waypoint);
    bool Land();

    void GazeboSubscriber(const gazebo_msgs::ModelStates::ConstPtr& msg);

    geometry_msgs::Twist GetVelocityRef();
    geometry_msgs::Pose Move(geometry_msgs::Twist refVelocity);

    DroneControl();

private:
    double lastError = 0.;
    double kp = 0.5;
    double ki = 1.;
};

#endif