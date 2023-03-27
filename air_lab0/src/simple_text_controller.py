#!/usr/bin/env python3
import math

# rosoy is the main API for ROS
import rospy

# import library with ros messages
import sensor_msgs.msg
import std_msgs.msg

#
# Class that contains the logic for our simple controller:
# - listen to the "text_cmd" topic, accept
# "forward", "backward", "left", "right" and "stop"
# as valid commands.
# - publish a wheel command on the "wheel_cmd" topic
#
class simple_text_controller:
    def __init__(self):
        # Subscribe to the "text_cmd" topic which has
        # "std_msgs/String" as message type.
        # It will call "self.command_callback" every
        # time a new message arrive
        self.command_sub = rospy.Subscriber("text_cmd",
        std_msgs.msg.String, self.command_callback)
        # Publisher to the "wheel_cmd" topic which has
        # "sensor_msgs/JointState" as message type.
        self.effort_pub = rospy.Publisher("wheel_cmd",
        sensor_msgs.msg.JointState, queue_size = 1)

    # Callback called everytime a new command message
    # is received
    def command_callback(self, command_msg):
        # Compute the effort depending on the received
        # command.
        if command_msg.data == "forward":
            left_effort = 0.5
            right_effort = 0.5
        elif command_msg.data == "left":
            left_effort = -0.5
            right_effort = 0.5
        elif command_msg.data == "backward":
            left_effort = -0.5
            right_effort = -0.5
        elif command_msg.data == "right":
            left_effort = 0.5
            right_effort = -0.5
        else:
            # anything else
            left_effort = 0.0
            right_effort = 0.0

        # Fill the effort_msg with the command values
        effort_msg = sensor_msgs.msg.JointState()
        effort_msg.name = ["left", "right"]
        effort_msg.effort = [left_effort, right_effort]
        self.effort_pub.publish(effort_msg)

if __name__ == '__main__':
    # Initialise the ROS sub system
    rospy.init_node('simple_text_controller', anonymous=False)
    # Create an instance of our controller
    ec = simple_text_controller()
    # Start listening to messages and loop forever
    rospy.spin()
