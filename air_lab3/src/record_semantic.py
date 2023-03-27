from std_msgs.msg import Empty, Float64
import time
import TstML
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist, PointStamped

from air_simple_sim_msgs.msg import SemanticObservation

from nav_msgs.msg import Path, Odometry
import numpy as np
import psutil
import signal
import rosbag
import subprocess
import json
from lrs_kdb_msgs.srv import QueryDatabase, InsertTriples
from visualization import visualize_db


class recordSemanticExecutor(TstML.Executor.AbstractNodeExecutor):
    def __init__(self, node, context):
        super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
        context)

        self.graphname = None
        self.p = None
        self.paused = False
        self.tf_listener = tf.TransformListener()

        time.sleep(2.0)

    def callback(self, data):

        uuid = data.uuid
        klass = data.klass

        stamp = data.point.header.stamp
        frame_id = data.point.header.frame_id

        self.tf_listener.waitForTransform(frame_id,"odom",stamp ,rospy.Duration(1.0))
        self.tf_listener.lookupTransform('odom', frame_id, rospy.Time(0))

        p=self.tf_listener.transformPoint("odom",data.point)

        graphnames = [self.graphname]

        query = "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>\
        PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>\
        \
        SELECT ?x ?y WHERE { <<someid>> a <<someklass>> ;\
                      properties:location [ gis:x ?x; gis:y ?y ] . }"

        query = query.replace("<someid>", uuid)
        query = query.replace("<someklass>", klass)

        bindings = ""
        service = ""

        serviceClient1 = rospy.ServiceProxy('/husky0/kDBServerNodelet/sparql_query', QueryDatabase)
        plan1 = serviceClient1(graphnames, "json", query, bindings, service)

        success = plan1.success
        result = plan1.result

        result_data = json.loads(result)

        if not result_data["results"]["bindings"]:
            content = "\
            @prefix classes: <http://www.ida.liu.se/~TDDE05/classes>\
            @prefix gis: <http://www.ida.liu.se/~TDDE05/gis>\
            @prefix properties: <http://www.ida.liu.se/~TDDE05/properties>\
            <someid> a <someclass>;\
                   properties:location [ gis:x <somex>; gis:y <somey> ] ."

            content = content.replace("someid", uuid)
            content = content.replace("someclass", klass)
            content = content.replace("<somex>", str(round(p.point.x,2)))
            content = content.replace("<somey>", str(round(p.point.y,2)))

            # insert the data with with service call and graphname.
            serviceClient2 = rospy.ServiceProxy('/husky0/kDBServerNodelet/insert_triples', InsertTriples)
            plan2 = serviceClient2(self.graphname, 'ttl', content)


    def start(self):
        if not self.paused:
            has_topic = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "topic")
            if(has_topic):
                topic = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "topic")
            else:
                print("Topic missing")
                topic = ""

            ### SET graphname ###
            has_graphname = self.node().hasParameter(
            TstML.TSTNode.ParameterType.Specific, "graphname")
            if(has_graphname):
                self.graphname = self.node().getParameter(
                TstML.TSTNode.ParameterType.Specific, "graphname")
            else:
                print("graphname missing")
                self.graphname = None

            # Here we say the type is SemanticObservation, is that wrong since the Subscriber
            # can take in any topic?
            self.semanticSub = rospy.Subscriber(topic, SemanticObservation, self.callback)

        return TstML.Executor.ExecutionStatus.Started()


    def pause(self):
        self.paused = True
        return TstML.Executor.ExecutionStatus.Paused()

    def resume(self):
        self.paused = False
        return TstML.Executor.ExecutionStatus.Running()

    def stop(self):
        self.semanticSub.unregister()
        return TstML.Executor.ExecutionStatus.Finished()

    def abort(self):
        return TstML.Executor.ExecutionStatus.Aborted()
