import std_msgs.msg
import time
import TstML
import rospy

class EchoExecutor(TstML.Executor.AbstractNodeExecutor):
  def __init__(self, node, context):
    super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
          context)
    self.pub = rospy.Publisher(node.getParameter(
          TstML.TSTNode.ParameterType.Specific,
          "topic"), std_msgs.msg.String, queue_size=1)
    self.count = 0
    self.paused = False

  def start(self):
    msg = std_msgs.msg.String()
    msg.data = self.node().getParameter(
          TstML.TSTNode.ParameterType.Specific, "text")

    finite_loop = self.node().hasParameter(
          TstML.TSTNode.ParameterType.Specific, "count")
    loop_count = self.node().getParameter(
          TstML.TSTNode.ParameterType.Specific, "count")

    # Called at a regular interval to publish on the topic
    def callback(event):
      if not self.paused:
        self.pub.publish(msg)
        self.count += 1
        if finite_loop and self.count >= loop_count:
          self.executionFinished(
              TstML.Executor.ExecutionStatus.Finished())
          self.timer.shutdown()

    # Create a timer
    self.timer = rospy.Timer(rospy.Duration(self.node().getParameter(
          TstML.TSTNode.ParameterType.Specific, "interval")),
          callback)

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
