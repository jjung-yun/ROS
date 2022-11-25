#!/usr/bin/env python

import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
from math import pi, sqrt, atan2

WAYPOINTS = [[5,0],[5,0.5],[0,0.5],[0,1],
             [5,1],[5,1.5],[0,1.5],[0,2],
             [5,2],[5,2.5],[0,2.5],[0,3]]

#####################################################################################################################################
class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D
#####################################################################################################################################
class turtlebot_move():
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0,0,0)  # initialization
        self.sensor = []

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.counter = 0
        self.trajectory = list()

        # track a sequence of waypoints
        for point in WAYPOINTS:
            self.move_to_point(point[0], point[1])
            rospy.sleep(1)
        self.stop()
        rospy.logwarn("Action done.")

        # plot trajectory
        data = np.array(self.trajectory)
        np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        plt.plot(data[:,0],data[:,1])
        plt.show()


    def move_to_point(self, x, y):
        # Compute orientation for angular vel and direction vector for linear vel
        diff_x = x - self.x
        diff_y = y - self.y
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector/sqrt(diff_x*diff_x + diff_y*diff_y)  # normalization
        theta = atan2(diff_y, diff_x)

        # We should adopt different parameters for different kinds of movement
        self.pid_theta.setPID(1, 0, 0)     # P control while steering
        self.pid_theta.setPoint(theta)
        rospy.logwarn("### PID: set target theta = " + str(theta) + " ###")

        # Adjust orientation first
        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2
            if abs(angular) < 0.01:
                break
            self.vel.linear.x = 0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        # Have a rest
        self.stop()
        self.pid_theta.setPoint(theta)
        self.pid_theta.setPID(1, 0.02, 0.2)  # PID control while moving

        # Move to the target point
        while not rospy.is_shutdown():
            if self.scan_wall(self.sensor) == 1:
                # Forward Obstacle Detection
                self.stop()
            else:
                diff_x = x - self.x
                diff_y = y - self.y
                vector = np.array([diff_x, diff_y])
                linear = np.dot(vector, direction_vector) # projection
                if abs(linear) > 0.2:
                    linear = linear/abs(linear)*0.2

                angular = self.pid_theta.update(self.theta)
                if abs(angular) > 0.2:
                    angular = angular/abs(angular)*0.2

                if abs(linear) < 0.01 and abs(angular) < 0.01:
                    break
                self.vel.linear.x = linear
                self.vel.angular.z = angular
                self.vel_pub.publish(self.vel)
                self.rate.sleep()
        self.stop()

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)
    
    #############################################################################################################################
    def lidar_callback(self, data):
        SensorData = data.ranges                            # Get the data from Lidar
        
        self.sensor = [self.avg(SensorData[12:15]), self.avg(SensorData[17:20]), self.avg(SensorData[22:25]),
                     self.avg(SensorData[27:30]), self.avg(SensorData[32:35]),
                     self.avg(SensorData[325:328]), self.avg(SensorData[330:333]), self.avg(SensorData[335:338]),
                     self.avg(SensorData[340:343]), self.avg(SensorData[345:348]), self.avg(SensorData[350:353])]
    
    def avg(self, data):
        return 	sum(data)/(len(data)-data.count(0)+0.01)
        
    def scan_wall(self, data):
        counting = 0
        for i in range(0, len(data)):
            if data[i] < 0.4 and data[i] > 0:	# Find the distances which are less 0.2m and counts the number of data 
                counting = counting+1
        if counting >= 4:						# If the count number of values over 6, turtlebot consider that there is wall
            return 1							# in front of It and returns 1			
        else:
            return 0
    #############################################################################################################################
    
    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Make messages saved and prompted in 5Hz rather than 100Hz
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])
            rospy.loginfo("odom: x=" + str(self.x) + ";  y=" + str(self.y) + ";  theta=" + str(self.theta))
#####################################################################################################################################
if __name__ == '__main__':
    try:
        turtlebot_move()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
#####################################################################################################################################