#!/usr/bin/env python3
from math import sqrt
import rospy
from rms.msg import *
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState
import time
import argparse
import numpy as np

class EKF:

    def __init__(self) -> None:
        
        self.robotName = "quadrotor"
        
        self.gpsData = [0., 0., 0.]
        self.gpsDataOld = []
        self.cyclesSinceLastGPSData = 0
        
        self.heightData = [0.]
        self.heightDataOld = []
        self.cyclesSinceLastHeightData = 0

        self.beaconPoses = {}
        self.beaconDists = {}
        self.beaconDistsOld = {}
        self.cyclesSinceLastBeaconData = {}

        self.state = 0
        self.freq = 10.
        q = 0.01
        rx = 9.
        ry = 9.
        rz = 25.
        rb = 1.
        rh = 0.01

        parser = argparse.ArgumentParser(description='Extended Kalman Filter')
        parser.add_argument('-model', type=str, default="quadrotor",
                            help='Robot name')
        parser.add_argument('-rx', type=float, default=9.0,
                            help='X gps squared variance')
        parser.add_argument('-ry', type=float, default=9.0,
                            help='Y gps squared variance')
        parser.add_argument('-rz', type=float, default=25.0,
                            help='Z gps squared variance')
        parser.add_argument('-rh', type=float, default=0.01,
                            help='Height sensor squared variance')
        parser.add_argument('-rb', type=float, default=1.0,
                            help='Radio beacons squared variance')
        parser.add_argument('-freq', type=float, default=10.0,
                            help='Frequency')
        args, unknown = parser.parse_known_args()

        self.robotName = args.model
        rx = args.rx
        ry = args.ry
        rz = args.rz
        rb = args.rb
        rh = args.rh
        self.freq = args.freq
        T = 1./self.freq

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

        print("Initializing EKF...")
        print("model  = {}".format(self.robotName))
        print("rx     = {}".format(rx))
        print("ry     = {}".format(ry))
        print("rz     = {}".format(rz))
        print("rh     = {}".format(rh))
        print("rb     = {}".format(rb))
        print("freq   = {}".format(self.freq))

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

            self.R = np.empty((0, 0), float)
            self.H = np.empty((0, 6), float)
            
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
            z = np.empty((0, 1), float)
            y = np.empty((0, 1), float) 

            if(len(self.gpsData) > 0): # If GPS data available
                z = np.vstack([z, np.matrix([ [self.gpsData[0]],
                                              [self.gpsData[1]],
                                              [self.gpsData[2]]])])

                self.H = np.vstack([self.H, np.matrix([ [1,  0,  0,  0,  0,  0],        
                                                        [0,  1,  0,  0,  0,  0],        
                                                        [0,  0,  1,  0,  0,  0] ])])    

                y = np.vstack([y, np.matrix([ [float(X_p[0][0])],      # xk-1
                                              [float(X_p[1][0])],      # yk-1
                                              [float(X_p[2][0])] ])])  # zk-1

                self.R = np.pad(self.R, ((0,3), (0,3)), mode='constant', constant_values=0)
                self.R[len(self.R)-3][len(self.R)-3] = rx
                self.R[len(self.R)-2][len(self.R)-2] = ry
                self.R[len(self.R)-1][len(self.R)-1] = rz

                if(len(self.gpsDataOld) > 0):
                    z = np.vstack([z, np.matrix([ [self.gpsDataOld[0]],
                                                  [self.gpsDataOld[1]],
                                                  [self.gpsDataOld[2]] ])])
                   
                    n = float(self.cyclesSinceLastGPSData)

                    self.H = np.vstack([self.H, np.matrix([ [1,  0,  0, -n*T,  0,   0],        
                                                            [0,  1,  0,   0, -n*T,  0],
                                                            [0,  0,  1,   0,   0, -n*T] ])])


                    y = np.vstack([y, np.matrix([ [float(X_p[0][0]) - float(X_p[3][0])*n*T],        # xk-1 - vxk-1·nT
                                                  [float(X_p[1][0]) - float(X_p[4][0])*n*T],        # yk-1 - vyk-1·nT
                                                  [float(X_p[2][0]) - float(X_p[5][0])*n*T] ])])    # zk-1 - vzk-1·nT

                    self.R = np.pad(self.R, ((0,3), (0,3)), mode='constant', constant_values=0)
                    self.R[len(self.R)-3][len(self.R)-3] = n*rx
                    self.R[len(self.R)-2][len(self.R)-2] = n*ry
                    self.R[len(self.R)-1][len(self.R)-1] = n*rz
                                     
                    self.cyclesSinceLastGPSData = 0

                self.gpsDataOld = self.gpsData.copy()

            self.gpsData.clear()
            self.cyclesSinceLastGPSData += 1          

            if(len(self.heightData) > 0): # If Height data available
                z = np.vstack([z, np.matrix([ [self.heightData[0]] ])])

                self.H = np.vstack([self.H, np.matrix([ [0,  0,  1,  0,  0,  0] ])])    

                y = np.vstack([y, np.matrix([ [float(X_p[2][0])] ])])  # zk-1

                self.R = np.pad(self.R, ((0,1), (0,1)), mode='constant', constant_values=0)
                self.R[len(self.R)-1][len(self.R)-1] = rh

                if(len(self.heightDataOld) > 0):
                    z = np.vstack([z, np.matrix([ [self.heightDataOld[0]] ])])
                   
                    n = float(self.cyclesSinceLastHeightData)

                    self.H = np.vstack([self.H, np.matrix([ [0,  0,  1,   0,   0, -n*T] ])])

                    y = np.vstack([y, np.matrix([ [float(X_p[2][0]) - float(X_p[5][0])*n*T] ])])    # zk-1 - vzk-1·nT

                    self.R = np.pad(self.R, ((0,1), (0,1)), mode='constant', constant_values=0)
                    self.R[len(self.R)-1][len(self.R)-1] = n*rh
                                     
                    self.cyclesSinceLastHeightData = 0

                self.heightDataOld = self.heightData.copy()

            self.heightData.clear()
            self.cyclesSinceLastHeightData += 1

            if(len(self.beaconDists) > 0): # If beacon data available
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
                    self.H = np.vstack([self.H, np.array([  (xk - xb)/(sqrt((xk-xb)**2 + (yk-yb)**2 + (zk-zb)**2)), 
                                                            (yk - yb)/(sqrt((xk-xb)**2 + (yk-yb)**2 + (zk-zb)**2)),
                                                            (zk - zb)/(sqrt((xk-xb)**2 + (yk-yb)**2 + (zk-zb)**2)), 0, 0, 0])])

                    y = np.vstack([y, np.matrix([sqrt((xk - xb)**2 + (yk - yb)**2 + (zk - zb)**2)])]) # sqrt((xk - xB)² + (yk - yB)² + (zk - zB)²)

                    self.R = np.pad(self.R, ((0,1), (0,1)), mode='constant', constant_values=0)
                    self.R[len(self.R)-1][len(self.R)-1] = rb

                    if key in self.beaconDistsOld:
                        z = np.vstack([z, np.matrix([self.beaconDistsOld[key]])])
                        n = float(self.cyclesSinceLastBeaconData[key])

                        self.H = np.vstack([self.H, np.array([  -(xb-xk+n*T*vx)/(sqrt((xb-xk+n*T*vx)**2 + (yb-yk+n*T*vy)**2 + (zb-zk+n*T*vz)**2)), 
                                                                -(yb-yk+n*T*vy)/(sqrt((xb-xk+n*T*vx)**2 + (yb-yk+n*T*vy)**2 + (zb-zk+n*T*vz)**2)),
                                                                -(zb-zk+n*T*vz)/(sqrt((xb-xk+n*T*vx)**2 + (yb-yk+n*T*vy)**2 + (zb-zk+n*T*vz)**2)),
                                                                (n*T*(xb-xk+n*T*vx))/(sqrt((xb-xk+n*T*vx)**2 + (yb-yk+n*T*vy)**2 + (zb-zk+n*T*vz)**2)),
                                                                (n*T*(yb-yk+n*T*vy))/(sqrt((xb-xk+n*T*vx)**2 + (yb-yk+n*T*vy)**2 + (zb-zk+n*T*vz)**2)),
                                                                (n*T*(zb-zk+n*T*vz))/(sqrt((xb-xk+n*T*vx)**2 + (yb-yk+n*T*vy)**2 + (zb-zk+n*T*vz)**2))])])

                        y = np.vstack([y, np.matrix([sqrt((xk -n*T*vx - xb)**2 + (yk -n*T*vy - yb)**2 + (zk -n*T*vz - zb)**2)])]) # sqrt((x-2*T*vx - xB)² + (y-2*T*vy - yB)² + (z-2*T*vz - zB)²)
                    
                        self.R = np.pad(self.R, ((0,1), (0,1)), mode='constant', constant_values=0)
                        self.R[len(self.R)-1][len(self.R)-1] = n*rb
                        self.cyclesSinceLastBeaconData[key] = 0

                    self.beaconDistsOld[key] = self.beaconDists[key]

            self.beaconDists.clear()
            for key, value in self.cyclesSinceLastBeaconData.items():
                self.cyclesSinceLastBeaconData[key] += 1

            y = z - y # Innovation matrix
            S = np.linalg.inv(np.matmul(np.matmul(self.H, P_p), np.transpose(self.H)) + self.R)
            K = np.matmul(np.matmul(P_p, np.transpose(self.H)), S)

            self.X = X_p + np.matmul(K, y)
            self.P = P_p - np.matmul(np.matmul(K, self.H), P_p)

            #

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
        self.gpsData = [0., 0., 0.]
        self.gpsData[0] = msg.x
        self.gpsData[1] = msg.y
        self.gpsData[2] = msg.z

    def onHeightMsg(self, msg):
        self.heightData = [0.]
        self.heightData[0] = msg.data

    def onBeaconMsg(self, msg):
        self.beaconDists[msg.id] = msg.dist

        if (msg.id not in self.cyclesSinceLastBeaconData):
            self.cyclesSinceLastBeaconData[msg.id] = 0

        if (msg.id not in self.beaconPoses):
            self.beaconPoses[msg.id] = (msg.point.x, msg.point.y, msg.point.z)

if __name__ == '__main__':
    ekf = EKF()