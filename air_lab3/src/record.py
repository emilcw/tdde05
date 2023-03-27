from std_msgs.msg import Empty, Float64
import time
import TstML
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
import numpy as np
import psutil
import signal
import rosbag
import subprocess


class recordExecutor(TstML.Executor.AbstractNodeExecutor):
    def __init__(self, node, context):
        super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
        context)

        self.p = None
        self.paused = False

        time.sleep(2.0)


    def start(self):
        if not self.paused:
            ### SET filename ###
            has_filename = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "filename")
            if(has_filename):
                filename = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "filename")
            else:
                print("Filename missing")
                filename = None

            ### SET Topics ###
            has_topics = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "topics")
            if(has_topics):
                topics = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "topics")
            else:
                print("Topics missing")
                topics = "hej"

            # Create command to command line

            cmd = ["rosbag", "record", "-O", filename]

            # Add topics to cmd
            if isinstance(topics, list):
                for topic in topics:
                    cmd.append(topic)
            else:
                cmd.append(topics)

            self.p = subprocess.Popen(cmd)

        return TstML.Executor.ExecutionStatus.Started()


    def terminate_process_and_children(self,p):
        """ Take a process as argument, and kill all the children
        recursively.
        """
        process = psutil.Process(self.p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        self.p.wait() # we wait for children to terminate

    def pause(self):
        self.paused = True
        return TstML.Executor.ExecutionStatus.Paused()

    def resume(self):
        self.paused = False
        return TstML.Executor.ExecutionStatus.Running()

    def stop(self):
        if(self.p != None):
            self.terminate_process_and_children(self.p)
        else:
            print("self.p was none!!")
        return TstML.Executor.ExecutionStatus.Finished()

    def abort(self):
        return TstML.Executor.ExecutionStatus.Aborted()
