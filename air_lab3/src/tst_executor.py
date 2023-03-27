#!/usr/bin/env python3
import math
import TstML
import rospkg
import TstML.Executor
import air_lab3.srv
from echo_executor import EchoExecutor
from drive_to import driveToExecutor
from explore import exploreExecutor
from record import recordExecutor
from record_semantic import recordSemanticExecutor
from std_srvs.srv import Empty
# rosoy is the main API for ROS
import rospy

# import library with ros messages
import sensor_msgs.msg
import std_msgs.msg


class tst_executor:
    def __init__(self):
        # Define class variables
        self.tst_registry = TstML.TSTNodeModelsRegistry()
        service = rospy.Service('execute_tst', air_lab3.srv.ExecuteTst , self.callback)

        abort_service = rospy.Service('abort', Empty , TstML.Executor.Executor.abort)
        stop_service = rospy.Service('stop', Empty , TstML.Executor.Executor.stop)
        pause_service = rospy.Service('pause', Empty , TstML.Executor.Executor.pause)
        resume_service = rospy.Service('resume', Empty , TstML.Executor.Executor.resume)

    # Callback called everytime a new command message
    # is received
    def callback(self, req):

        filename = req.tst_file
        self.tst_registry.loadDirectory(rospkg.RosPack().get_path("air_tst") + "/configs")
        tst_node = TstML.TSTNode.load(filename, self.tst_registry)

         # Create a registry with node executors
        tst_executor_registry = TstML.Executor.NodeExecutorRegistry()

          # Setup the executors for sequence and concurrent
        tst_executor_registry.registerNodeExecutor(
        self.tst_registry.model("seq"),
        TstML.Executor.DefaultNodeExecutor.Sequence)

        tst_executor_registry.registerNodeExecutor(
        self.tst_registry.model("conc"),
        TstML.Executor.DefaultNodeExecutor.Concurrent)

        tst_executor_registry.registerNodeExecutor(
        self.tst_registry.model("echo"), EchoExecutor)

        tst_executor_registry.registerNodeExecutor(
        self.tst_registry.model("drive-to"), driveToExecutor)

        tst_executor_registry.registerNodeExecutor(
        self.tst_registry.model("explore"), exploreExecutor)

        tst_executor_registry.registerNodeExecutor(
        self.tst_registry.model("record"), recordExecutor)

        tst_executor_registry.registerNodeExecutor(
        self.tst_registry.model("record-semantic"), recordSemanticExecutor)

         # Create an executor using the executors defined
         # in tst_executor_registry
        tst_executor = TstML.Executor.Executor(tst_node, tst_executor_registry)

         # Start execution
        tst_executor.start()

         # Block until the execution has finished
        status = tst_executor.waitForFinished()

         # Display the result of execution
        if status.type() == TstML.Executor.ExecutionStatus.Type.Finished:
            print("Execution successful")
            success = True
            error_message = ""
        else:
            success = False
            error_message = status.message()
            print("Execution failed: {}".format(status.message()))


        return air_lab3.srv.ExecuteTstResponse(success, error_message)


if __name__ == '__main__':
    # Initialise the ROS sub system
    rospy.init_node('tst_executor', anonymous=False)
    # Create an instance of our controller
    et = tst_executor()
    # Start listening to messages and loop forever
    rospy.spin()
