#!/usr/bin/env python3

import rospy
import nav_msgs.msg
import geometry_msgs.msg
import air_lab1.msg
import math

# C++ API is very similar to the Python API and therefore the explanation for C++ apply to Python as well.

class evaluate_controller:
    def __init__(self):
        self.odometry_sub =  rospy.Subscriber("odometry", nav_msgs.msg.Odometry, self.odometry_callback)
        self.velocity_sub =  rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, self.desired_velocity_callback)

        self.stat_pub = rospy.Publisher("evaluation_stat", air_lab1.msg.ControllerEvaluationStat, queue_size = 1)

        self.last_desired_velocity = None
        self.samples        = 0.0
        self.average_error  = 0.0
    def odometry_callback(self, odometry):
        if(self.last_desired_velocity):
          last_error = (odometry.twist.twist.linear.x - self.last_desired_velocity.linear.x) ** 2 + (odometry.twist.twist.angular.z - self.last_desired_velocity.angular.z) ** 2
          self.average_error = self.average_error * self.samples + last_error
          self.samples += 1.0
          self.average_error /= self.samples
          msg = air_lab1.msg.ControllerEvaluationStat()
          msg.last_error = math.sqrt(last_error)
          msg.average_error = math.sqrt(self.average_error)
          msg.samples = int(self.samples)
          self.stat_pub.publish(msg)

    def desired_velocity_callback(self, velocity):
        self.last_desired_velocity = velocity

if __name__ == '__main__':
    rospy.init_node('evaluate_controller', anonymous=False)
    ec = evaluate_controller()
    rospy.spin()
