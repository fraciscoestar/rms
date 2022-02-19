#!/usr/bin/env python3
from cmath import sqrt
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
        self.beaconDistsOld = {}

        self.state = 0
        self.freq = 10.
        T = 1./self.freq
        q = 0.001
        rx = 9.
        ry = 9.
        rz = 25.
        rb = 9.
        rh = 0.025

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
                            [0,  0,  0,  0,  0,  rz]])

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

            self.R = np.matrix([[rx, 0,  0,  0,  0,  0,  0,  0],
                                [0,  ry, 0,  0,  0,  0,  0,  0],
                                [0,  0,  rz, 0,  0,  0,  0,  0],
                                [0,  0,  0,  rx, 0,  0,  0,  0],
                                [0,  0,  0,  0,  ry, 0,  0,  0],
                                [0,  0,  0,  0,  0,  rz, 0,  0],
                                [0,  0,  0,  0,  0,  0,  rh, 0],
                                [0,  0,  0,  0,  0,  0,  0,  rh]])

            self.H = np.matrix([[1,  0,  0,  0,  0,  0],
                                [0,  1,  0,  0,  0,  0],
                                [0,  0,  1,  0,  0,  0],
                                [1,  0,  0, -T,  0,  0],
                                [0,  1,  0,  0, -T,  0],
                                [0,  0,  1,  0,  0, -T],
                                [0,  0,  1,  0,  0,  0],
                                [0,  0,  1,  0,  0, -T]])
            
            # Model selection
            if (self.state == 0 or self.state == 2): # If landed or idle no movement is predicted and velocity is zero
                self.F = np.matrix([[1,  0,  0,  0,  0,  0],
                                    [0,  1,  0,  0,  0,  0],
                                    [0,  0,  1,  0,  0,  0],
                                    [0,  0,  0,  0,  0,  0],
                                    [0,  0,  0,  0,  0,  0],
                                    [0,  0,  0,  0,  0,  0]])
            elif (self.state == 1 or self.state == 4): # If taking off or landing, only movement in z is predicted with constant vz
                self.F = np.matrix([[1,  0,  0,  0,  0,  0],
                                    [0,  1,  0,  0,  0,  0],
                                    [0,  0,  1,  0,  0,  T],
                                    [0,  0,  0,  0,  0,  0],
                                    [0,  0,  0,  0,  0,  0],
                                    [0,  0,  0,  0,  0,  1]])
            else: # If moving, suppose movement in all axes at constant speed
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
            z = np.matrix([[self.gpsData[0]],
                           [self.gpsData[1]],
                           [self.gpsData[2]],
                           [self.gpsDataOld[0]],
                           [self.gpsDataOld[1]],
                           [self.gpsDataOld[2]],
                           [self.heightData],
                           [self.heightDataOld]])

            y = np.matrix([[0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.],
                            [0.]]) # Innovation matrix
            y[0][0] = X_p[0][0] # xk-1
            y[1][0] = X_p[1][0] # yk-1
            y[2][0] = X_p[2][0] # zk-1
            y[3][0] = X_p[0][0] - X_p[3][0]*T # xk-1 - vxk-1·T
            y[4][0] = X_p[1][0] - X_p[4][0]*T # yk-1 - vyk-1·T
            y[5][0] = X_p[2][0] - X_p[5][0]*T # zk-1 - vzk-1·T
            y[6][0] = X_p[2][0] # zk-1
            y[7][0] = X_p[2][0] # zk-1 - vzk-1·T

            if(self.beaconDists): # If beacons
                for key, value in self.beaconDists.items():
                    z = np.vstack([z, np.matrix([value])])
                    
                    xk = float(X_p[0][0])
                    yk = float(X_p[1][0])
                    zk = float(X_p[2][0])
                    vx = float(X_p[3][0])
                    vy = float(X_p[4][0])
                    vz = float(X_p[5][0])
                    xb = float((self.beaconPoses[key])[0])
                    yb = float((self.beaconPoses[key])[1])
                    zb = float((self.beaconPoses[key])[2])
                    self.H = np.vstack([self.H, np.array([  np.real((xk - xb)/(sqrt((xk-xb)**2 + (yk-yb)**2 + (zk-zb)**2))), 
                                                            np.real((yk - yb)/(sqrt((xk-xb)**2 + (yk-yb)**2 + (zk-zb)**2))),
                                                            np.real((zk - zb)/(sqrt((xk-xb)**2 + (yk-yb)**2 + (zk-zb)**2))), 0, 0, 0])])

                    y = np.vstack([y, np.matrix([sqrt((xk - xb)**2 + (yk - yb)**2 + (zk - zb)**2)])]) # sqrt((xk - xB)² + (yk - yB)² + (zk - zB)²)

                    if key in self.beaconDistsOld:
                        z = np.vstack([z, np.matrix([self.beaconDistsOld[key]])])

                        self.H = np.vstack([self.H, np.array([  np.real(-(xb-xk+2*T*vx)/(sqrt((xb-xk+2*T*vx)**2 + (yb-yk+2*T*vy)**2 + (zb-zk+2*T*vz)**2))), 
                                                                np.real(-(yb-yk+2*T*vy)/(sqrt((xb-xk+2*T*vx)**2 + (yb-yk+2*T*vy)**2 + (zb-zk+2*T*vz)**2))),
                                                                np.real(-(zb-zk+2*T*vz)/(sqrt((xb-xk+2*T*vx)**2 + (yb-yk+2*T*vy)**2 + (zb-zk+2*T*vz)**2))),
                                                                np.real((2*T*(xb-xk+2*T*vx))/(sqrt((xb-xk+2*T*vx)**2 + (yb-yk+2*T*vy)**2 + (zb-zk+2*T*vz)**2))),
                                                                np.real((2*T*(yb-yk+2*T*vy))/(sqrt((xb-xk+2*T*vx)**2 + (yb-yk+2*T*vy)**2 + (zb-zk+2*T*vz)**2))),
                                                                np.real((2*T*(zb-zk+2*T*vz))/(sqrt((xb-xk+2*T*vx)**2 + (yb-yk+2*T*vy)**2 + (zb-zk+2*T*vz)**2)))])])

                        y = np.vstack([y, np.matrix([sqrt((xk -2*T*vx - xb)**2 + (yk -2*T*vy - yb)**2 + (zk -2*T*vz - zb)**2)])]) # sqrt((x-2*T*vx - xB)² + (y-2*T*vy - yB)² + (z-2*T*vz - zB)²)
                    
                        self.R = np.pad(self.R, ((0,1), (0,1)), mode='constant', constant_values=0)
                        self.R[len(self.R)-1][len(self.R)-1] = rb

                    self.R = np.pad(self.R, ((0,1), (0,1)), mode='constant', constant_values=0)
                    self.R[len(self.R)-1][len(self.R)-1] = rb
                    self.beaconDistsOld[key] = self.beaconDists[key]

            y = z - y
            S = np.linalg.inv(np.matmul(np.matmul(self.H, P_p), np.transpose(self.H)) + self.R)
            K = np.matmul(np.matmul(P_p, np.transpose(self.H)), S)

            self.X = np.real(X_p + np.matmul(K, y))
            self.P = P_p - np.matmul(np.matmul(K, self.H), P_p)

            #

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

            self.beaconDists.clear()
            
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