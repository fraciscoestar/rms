#!/usr/bin/env python
from math import nan
from operator import matmul
import rospy
from rms.msg import *
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState
import time
import numpy as np

class EKF:

    def __init__(self) -> None:
        
        self.robotName = "quadrotor"
        
        self.gpsData = [0., 0., 0.]
        self.gpsDataOld = [0., 0., 0.]
        self.heightData = 0.
        self.heightDataOld = 0.

        self.beaconPoses = {}
        self.beaconDists = {}

        self.state = 0
        self.freq = 10.
        T = 1./self.freq
        q = 0.01
        rx = 10.
        ry = 10.
        rz = 30.
        rh = 0.05

        self.F = np.matrix([[1,  0,  0,  T,  0,  0],
                            [0,  1,  0,  0,  T,  0],
                            [0,  0,  1,  0,  0,  T],
                            [0,  0,  0,  1,  0,  0],
                            [0,  0,  0,  0,  1,  0],
                            [0,  0,  0,  0,  0,  1]])

        self.H = np.matrix([[1,  0,  0,  0,  0,  0],
                            [0,  1,  0,  0,  0,  0],
                            [0,  0,  1,  0,  0,  0],
                            [1,  0,  0, -T,  0,  0],
                            [0,  1,  0,  0, -T,  0],
                            [0,  0,  1,  0,  0, -T],
                            [0,  0,  1,  0,  0,  0],
                            [0,  0,  1,  0,  0, -T]])

        self.Q = np.matrix([[q,  0,  0,  0,  0,  0],
                            [0,  q,  0,  0,  0,  0],
                            [0,  0,  q,  0,  0,  0],
                            [0,  0,  0,  q,  0,  0],
                            [0,  0,  0,  0,  q,  0],
                            [0,  0,  0,  0,  0,  q]])

        self.R = np.matrix([[rx, 0,  0,  0,  0,  0,  0,  0],
                            [0,  ry, 0,  0,  0,  0,  0,  0],
                            [0,  0,  rz, 0,  0,  0,  0,  0],
                            [0,  0,  0,  rx, 0,  0,  0,  0],
                            [0,  0,  0,  0,  ry, 0,  0,  0],
                            [0,  0,  0,  0,  0,  rz, 0,  0],
                            [0,  0,  0,  0,  0,  0,  rh, 0],
                            [0,  0,  0,  0,  0,  0,  0,  rh]])

        self.P = np.matrix([[rx, 0,  0,  0,  0,  0],
                            [0,  ry, 0,  0,  0,  0],
                            [0,  0,  rz, 0,  0,  0],
                            [0,  0,  0,  rx, 0,  0],
                            [0,  0,  0,  0,  ry, 0],
                            [0,  0,  0,  0,  0,  rz] ])

        self.X = np.matrix([[0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.]])

        rospy.init_node('EKF')

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        time.sleep(0.5)

        rospy.Subscriber("/beacon", BeaconMsg, self.onBeaconMsg)
        rospy.Subscriber("/gps", Point, self.onGPSMsg)
        rospy.Subscriber("/height", Float64, self.onHeightMsg)
        rospy.Subscriber("/drone_state", Int32, self.onDroneStateMsg)

        self.pub = rospy.Publisher("/ekf", ModelState, queue_size=10)

        self.rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            
            # Model selection
            if (self.state == 0 or self.state == 2): # Static in both cases
                self.F = np.matrix([[1,  0,  0,  0,  0,  0],
                                    [0,  1,  0,  0,  0,  0],
                                    [0,  0,  1,  0,  0,  0],
                                    [0,  0,  0,  1,  0,  0],
                                    [0,  0,  0,  0,  1,  0],
                                    [0,  0,  0,  0,  0,  1]])
            else:
                self.F = np.matrix([[1,  0,  0,  T,  0,  0],
                                    [0,  1,  0,  0,  T,  0],
                                    [0,  0,  1,  0,  0,  T],
                                    [0,  0,  0,  1,  0,  0],
                                    [0,  0,  0,  0,  1,  0],
                                    [0,  0,  0,  0,  0,  1]])

            # PREDICTION
            X_p = np.matmul(self.F, self.X)
            P_p = np.matmul(np.matmul(self.F, self.P), np.transpose(self.F)) + self.Q

            # UPDATE
            temp = np.linalg.inv(np.matmul(np.matmul(self.H, P_p), np.transpose(self.H)) + self.R)
            K = np.matmul(np.matmul(P_p, np.transpose(self.H)), temp)

            z = np.matrix([[self.gpsData[0]],
                           [self.gpsData[1]],
                           [self.gpsData[2]],
                           [self.gpsDataOld[0]],
                           [self.gpsDataOld[1]],
                           [self.gpsDataOld[2]],
                           [self.heightData],
                           [self.heightDataOld] ])

            self.X = X_p + np.matmul(K, z - matmul(self.H, X_p))
            self.P = P_p - np.matmul(np.matmul(K, self.H), P_p)

            self.heightDataOld = self.heightData
            self.gpsDataOld = self.gpsData

            msg = ModelState()
            msg.model_name = self.robotName
            msg.pose.position.x = self.X[0][0]
            msg.pose.position.y = self.X[1][0]
            msg.pose.position.z = self.X[2][0]
            msg.twist.linear.x = self.X[3][0]
            msg.twist.linear.y = self.X[4][0]
            msg.twist.linear.z = self.X[5][0]
            self.pub.publish(msg)
            
            self.rate.sleep()

    def onDroneStateMsg(self, msg):
        self.state = msg.data

    def onGPSMsg(self, msg):
        self.gpsData[0] = msg.x
        self.gpsData[1] = msg.y
        self.gpsData[2] = msg.z

    def onHeightMsg(self, msg):
        self.heightData = msg.data

    def onBeaconMsg(self, msg):
        self.beaconDists[msg.id] = msg.dist

        if (msg.id not in self.beaconPoses):
            self.beaconPoses[msg.id] = (msg.point.x, msg.point.y, msg.point.z)

if __name__ == '__main__':
    ekf = EKF()