#!/usr/bin/env python3
import rospy
import sys

import std_msgs.msg
import geometry_msgs.msg 
import air_lab1.msg

import subprocess
import threading

class exhaustive_config:
  def __init__(self):
    pass
  def next_step(self, tuner):
    tuner.increment_count = 50
    if tuner.value_index == 0:
      tuner.start = 0
      tuner.end   = 1.0
    elif tuner.value_index == 1:
      tuner.start = 0
      tuner.end   = 0.01
    elif tuner.value_index == 2:
      tuner.start = 0
      tuner.end   = 0.001
      
class student_config:
  def __init__(self):
    pass
  def next_step(self, tuner):
    tuner.increment_count = 10
    if tuner.value_index == 0:
      tuner.start = 0.4
      tuner.end   = 0.8
    elif tuner.value_index == 1:
      tuner.start = 0.008
      tuner.end   = 0.01
    elif tuner.value_index == 2:
      tuner.start = 0.0008
      tuner.end   = 0.001



class auto_tune:
  def __init__(self, config):
    self.config = config
    self.evaluation_stat_sub = rospy.Subscriber("evaluation_stat", air_lab1.msg.ControllerEvaluationStat, self.evaluation_stat_callback)
    self.velocity_pub =  rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size = 1)
    self.to_vel_ctrl_pub = rospy.Publisher("to_vel_control", std_msgs.msg.Empty, queue_size = 1)
    
    self.increment_count = 40
    
    self.start = 0.2
    self.end = 1.0
    self.value_index = 0
    self.config.next_step(self)
    
    
    self.best_values = [0, 0, 0]
    self.lowest_error = 1e999
    
    self.initialise_values()
    
    self.sum_error = 0
    
    self.init_step_time = 2.0
    self.control_step_time = 5.0
    self.received_messages = 0
    
    self.step0()
  def initialise_values(self):
    self.increment = (self.end - self.start) / self.increment_count
    self.values = list(self.best_values)
    self.values[self.value_index] = self.start
    if self.start == 0:
      self.values[self.value_index] += self.increment_count
    
    
  def evaluation_stat_callback(self, msg):
    self.sum_error += msg.last_error
    self.received_messages += 1
    
  def stop_controller(self):
    self.controller_process.kill()
    
  def step0(self):
    print("Start testing: {}".format(self.values))
    self.controller_process = subprocess.Popen(["rosrun", "nodelet", "nodelet", "standalone", "air_lab1/husky_control_node", "__ns:=/husky0", "_Kp:={}".format(self.values[0]), "_Kd:={}".format(self.values[1]), "_Ki:={}".format(self.values[2])])
    self.timer = threading.Timer(self.init_step_time, self.step1)
    self.timer.start()

  def step1(self):
    print("Send linear velocity")
    self.to_vel_ctrl_pub.publish(std_msgs.msg.Empty())
    self.send_velocity(0.5, 0.5)
    self.sum_error = 0
    self.received_messages = 0
    self.timer = threading.Timer(self.control_step_time, self.step2)
    self.timer.start()

  def step2(self):
    print("Send null linear velocity")
    self.send_velocity(0.0, 0.0)
    self.timer = threading.Timer(self.control_step_time, self.step3)
    self.timer.start()

  def step3(self):
    print("Finish up")
    #terminate_process_and_children(self.controller_process)
    if self.received_messages == 0:
      raise RuntimeError("Did not received any message from evaluate_controller! Check the script is running properly.")
    self.controller_process.kill()
    print("Error for {} was {}".format(self.values, self.sum_error))
    if(self.sum_error < self.lowest_error):
      self.lowest_error = self.sum_error
      self.best_values = list(self.values)
      print("New lowest error!")
    self.values[self.value_index] += self.increment
    if self.values[self.value_index] > self.end:
      if self.value_index == 0:
        self.value_index = 1
        self.config.next_step(self)
      elif self.value_index == 1:
        self.value_index = 2
        self.config.next_step(self)
      else:
        print("Best values are [Kp, Kd, Ki] = {} with error {}".format(self.best_values, self.lowest_error))
        rospy.signal_shutdown("Timed out waiting for Action Server")
        sys.exit(1)     
      
      self.initialise_values()
    self.timer = threading.Timer(self.init_step_time, self.step0)
    self.timer.start()

  def send_velocity(self, linear, angular):
    vel = geometry_msgs.msg.Twist()
    vel.linear.x = linear
    vel.angular.z = angular
    self.velocity_pub.publish(vel)

if __name__ == '__main__':
    rospy.init_node('autotune', anonymous=False)
    at = auto_tune(student_config())
    rospy.spin()
    #while not rospy.is_shutdown():
        #obj.data = 536
        #publisherobj.publish(obj)
        #rospy.sleep(1)
