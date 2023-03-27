from std_msgs.msg import Empty, Float64
import time
import TstML
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
import numpy as np


class exploreExecutor(TstML.Executor.AbstractNodeExecutor):
    def __init__(self, node, context):
        super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
        context)

        self.pub_way_control = rospy.Publisher('/husky0/to_waypoints_control', Empty, queue_size=1)
        self.pub_cmd_waypoints = rospy.Publisher('/husky0/cmd_waypoints', Path, queue_size=1)

        self.curr_posx = 0
        self.curr_posy = 0

        self.waypointSub = rospy.Subscriber('/husky0/waypoints_finished', Empty, self.callback)
        self.pub_max_velocity = rospy.Publisher('/husky0/max_velocity', Twist, queue_size=1)

        self.odometrySub = rospy.Subscriber('/husky0/odometry', Odometry, self.setvalues)

        self.paused = False

        time.sleep(2.0)

    def setvalues(self,data):
        self.curr_posx = data.pose.pose.position.x
        self.curr_posy = data.pose.pose.position.y

    def callback(self, event):
        self.executionFinished(TstML.Executor.ExecutionStatus.Finished())
        print("Finished execution explore")


    def start(self):
        if not self.paused:

            # Set initial speed for robot
            max_speed = 0.5
            angular_speed = 3*max_speed
            linear_speed = max_speed

            move_cmd = Twist()
            move_cmd.linear.x = linear_speed
            move_cmd.angular.z = angular_speed

            self.pub_max_velocity.publish(move_cmd)


            ### SET RADIUS ###
            has_radius = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "radius")
            if(has_radius):
                radius = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "radius")
            else:
                radius = 1


            ### SET a ###
            has_a = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "a")
            if(has_a):
                a = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "a")
            else:
                a = 0

            ### SET b ###
            has_b = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "b")
            if(has_b):
                b = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "b")
            else:
                b = 1

            theta = 0
            r = 0
            path = Path()
            path.header.frame_id = "odom"

            while(r < radius):
                quat = tf.transformations.quaternion_from_euler(0, 0, theta + np.pi/2)
                r = a + b * theta

                pose = PoseStamped()
                pose.pose.position.x = self.curr_posx + (r * np.cos(theta))
                pose.pose.position.y = self.curr_posy + (r * np.sin(theta))
                pose.pose.position.z = 0

                #print("(", self.curr_posx + (r*np.cos(theta)), ",", self.curr_posy+(r*np.sin(theta)), ")")

                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]

                theta += np.pi/4
                path.poses.append(pose)

            msg = Empty()
            self.pub_way_control.publish(msg)
            time.sleep(1)
            self.pub_cmd_waypoints.publish(path)

        return TstML.Executor.ExecutionStatus.Started()

    def pause(self):
        self.paused = True
        return TstML.Executor.ExecutionStatus.Paused()
    def resume(self):
        self.paused = False
        return TstML.Executor.ExecutionStatus.Running()
    def stop(self):
        #self.timer.shutdown()
        return TstML.Executor.ExecutionStatus.Finished()
    def abort(self):
        #self.timer.shutdown()
        return TstML.Executor.ExecutionStatus.Aborted()
