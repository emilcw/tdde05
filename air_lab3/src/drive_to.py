from std_msgs.msg import Empty, Float64
import time
import TstML
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist


class driveToExecutor(TstML.Executor.AbstractNodeExecutor):
    def __init__(self, node, context):
        super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
        context)
        self.pub_pos_control = rospy.Publisher('/husky0/to_position_control', Empty, queue_size=1)
        self.pub_way_control = rospy.Publisher('/husky0/to_waypoints_control', Empty, queue_size=1)

        self.pub_cmd_pos = rospy.Publisher('/husky0/cmd_position', PoseStamped, queue_size=1)
        self.pub_dest = rospy.Publisher('/husky0/destination', PoseStamped, queue_size=1)

        self.positionSub = rospy.Subscriber('/husky0/position_reached', Empty, self.callback)
        self.waypointSub = rospy.Subscriber('/husky0/waypoints_finished', Empty, self.callback)

        self.pub_max_velocity = rospy.Publisher('/husky0/max_velocity', Twist, queue_size=1)

        self.paused = False

        time.sleep(2.0)

    def callback(self, event):
        self.executionFinished(TstML.Executor.ExecutionStatus.Finished())
        print("Finished execution drive to")


    def start(self):
        if not self.paused:
            point = self.node().getParameter(
            TstML.TSTNode.ParameterType.Specific, "p")

            ### SET MAX SPEED ###
            has_max_speed = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "maximum-speed")
            if(has_max_speed):
                max_speed = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "maximum-speed")
            else:
                max_speed = 0.5

            angular_speed = 3*max_speed
            linear_speed = max_speed

            move_cmd = Twist()
            move_cmd.linear.x = linear_speed
            move_cmd.angular.z = angular_speed

            self.pub_max_velocity.publish(move_cmd)

            ### SET YAW ###
            has_yaw = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "heading")
            if(has_yaw):
                yaw = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "heading")
            else:
                yaw = 0

            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            # quat is an array with the coordinates as [x,y,z,w]

            ### SET MOTION PLANNER ###
            has_mp = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "use-motion-planner")
            if(has_mp):
                use_mp = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "use-motion-planner")
            else:
                use_mp = False

            msg = Empty()
            pose = PoseStamped()
            pose.pose.position.x = point['x']
            pose.pose.position.y = point['y']
            pose.pose.position.z = point['z']

            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            if(use_mp):
                #Waypoint mode
                self.pub_way_control.publish(msg)
                time.sleep(1)
                self.pub_dest.publish(pose)

            else:
                #Position mode
                self.pub_pos_control.publish(msg)
                time.sleep(1)
                self.pub_cmd_pos.publish(pose)

        return TstML.Executor.ExecutionStatus.Started()

    def pause(self):
        self.paused = True
        return TstML.Executor.ExecutionStatus.Paused()
    def resume(self):
        self.paused = False
        return TstML.Executor.ExecutionStatus.Running()
    def stop(self):
        self.timer.shutdown()
        return TstML.Executor.ExecutionStatus.Finished()
    def abort(self):
        self.timer.shutdown()
        return TstML.Executor.ExecutionStatus.Aborted()
