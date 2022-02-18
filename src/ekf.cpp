#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <rms/BeaconMsg.h>

double* measurements; // Format: [xgps, ygps, zgps, zh, d1, d2, d3, ..., dn]' need to preallocate space
geometry_msgs::Point* beaconPoints;

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

void FillArrayWithNaN(double* array)
{
    for (int i = 0; i < 20; i++)
    {
        array[i] = std::nan("");
    }
}

int main(int argc, char* argv[])
{
    measurements = new double[20]; // Reserve 20 spaces -> Nmax=16
    beaconPoints = new geometry_msgs::Point[16]; // N spaces for N beacons

    FillArrayWithNaN(measurements);

    double freq = 2.;
    double T = 1./freq;

    double q = 0.1;
    double rx = 0.5;
    double ry = 0.5;
    double rz = 1;
    double rh = 0.1;

    double F[6][6] = { {1,  0,  0,  T,  0,  0},
                       {0,  1,  0,  0,  T,  0},
                       {0,  0,  1,  0,  0,  T},
                       {0,  0,  0,  1,  0,  0},
                       {0,  0,  0,  0,  1,  0},
                       {0,  0,  0,  0,  0,  1} };

    double H[8][6] = { {1,  0,  0,  0,  0,  0},
                       {0,  1,  0,  0,  0,  0},
                       {0,  0,  1,  0,  0,  0},
                       {1,  0,  0, -T,  0,  0},
                       {0,  1,  0,  0, -T,  0},
                       {0,  0,  1,  0,  0, -T},
                       {0,  0,  1,  0,  0,  0},
                       {0,  0,  1,  0,  0, -T} };

    double Q[6][6] = { {q,  0,  0,  0,  0,  0},
                       {0,  q,  0,  0,  0,  0},
                       {0,  0,  q,  0,  0,  0},
                       {0,  0,  0,  q,  0,  0},
                       {0,  0,  0,  0,  q,  0},
                       {0,  0,  0,  0,  0,  q} };

    double R[8][8] = { {rx, 0,  0,  0,  0,  0,  0,  0},
                       {0,  ry, 0,  0,  0,  0,  0,  0},
                       {0,  0,  rz, 0,  0,  0,  0,  0},
                       {0,  0,  0,  rx, 0,  0,  0,  0},
                       {0,  0,  0,  0,  ry, 0,  0,  0},
                       {0,  0,  0,  0,  0,  rz, 0,  0},
                       {0,  0,  0,  0,  0,  0,  rh, 0},
                       {0,  0,  0,  0,  0,  0,  0,  rh} };

    double P[6][6] = { {rx, 0,  0,  0,  0,  0},
                       {0,  ry, 0,  0,  0,  0},
                       {0,  0,  rz, 0,  0,  0},
                       {0,  0,  0,  rx, 0,  0},
                       {0,  0,  0,  0,  ry, 0},
                       {0,  0,  0,  0,  0,  rz} };

    double X[6][1] = { {0},
                       {0},
                       {0},
                       {0},
                       {0},
                       {0} };

    ros::init(argc, argv, "extended_kalman_filter");
    ros::NodeHandle nh;

    ros::service::waitForService("/gazebo/spawn_sdf_model");

    ros::Subscriber gpsSubscriber = nh.subscribe<geometry_msgs::Point>("/gps", 10, GPSCallback);
    ros::Subscriber heightSubscriber = nh.subscribe<std_msgs::Float64>("/height", 10, HeightCallback);
    ros::Subscriber beaconSubscriber = nh.subscribe<rms::BeaconMsg>("/beacon", 10, BeaconCallback);

    ros::Rate rate(freq);
    while(ros::ok())
    {
        FillArrayWithNaN(measurements);
        ros::spinOnce();

        // PREDICTION

        double X_p[6][1];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                X_p[i][j] = 0.;
                for (int k = 0; k < 6; k++)
                {
                    X_p[i][j] += F[i][k] * X[k][j];    
                }                
            }           
        }

        
        
        
        rate.sleep();
    }
}