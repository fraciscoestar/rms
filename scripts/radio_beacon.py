#!/usr/bin/env python
from dis import dis
import time
import rospy
import math
import argparse
import numpy as np
from gazebo_msgs.msg import ModelState, ModelStates
from std_msgs.msg import Float64

class RadioBeacon():

    def onModelStates(self, msg):
        for i in range(len(msg.name)):
            if (msg.name[i] == self.robotName):
                xRobot = msg.pose[i].position.x
                yRobot = msg.pose[i].position.y
                zRobot = msg.pose[i].position.z
                dist = math.sqrt(pow(self.x - xRobot, 2) + pow(self.y - yRobot, 2) + pow(self.z - zRobot, 2))
                noise = np.random.normal(self.mean, self.stddev)
                self.pub.publish(noise)

    def main(self):
        parser = argparse.ArgumentParser(description='Radio Beacon')
        parser.add_argument('-id', type=int, default=1,
                            help='beacon id')
        parser.add_argument('-x', type=float, default=0.0,
                            help='initial x position')
        parser.add_argument('-y', type=float, default=0.0,
                            help='initial y position')
        parser.add_argument('-z', type=float, default=0.0,
                            help='initial z position')
        parser.add_argument('-mean', type=float, default=0.0,
                            help='AWGN mean')
        parser.add_argument('-stddev', type=float, default=0.0,
                            help='AWGN standard deviation')
        args, unknown = parser.parse_known_args()

        self.x = args.x
        self.y = args.y
        self.z = args.z
        self.mean = args.mean
        self.stddev = args.stddev

        # Init ros node
        rospy.init_node('radio_beacon_{}'.format(args.id))

        self.robotName = "quadrotor"
        self.pub = rospy.Publisher('/beacon', Float64, queue_size=10)

        # Sleep for waiting the world to load
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        time.sleep(1.0)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.onModelStates)

        rospy.spin()

if __name__ == "__main__":
    RB = RadioBeacon()
    RB.main()