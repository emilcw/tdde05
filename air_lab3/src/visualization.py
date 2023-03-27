#!/usr/bin/env python3
import rospy
import visualization_msgs.msg
import geometry_msgs.msg
import std_msgs

from std_msgs.msg import Empty, Float64
import time
import TstML
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist

from air_simple_sim_msgs.msg import SemanticObservation

from nav_msgs.msg import Path, Odometry
import numpy as np
import psutil
import signal
import rosbag
import subprocess
import json
from lrs_kdb_msgs.srv import QueryDatabase, InsertTriples

def visualize_db(graphname):

    query_all = "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>\
    PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>\
    \
    SELECT ?x ?y ?id ?class WHERE { ?id a ?class ;\
                  properties:location [ gis:x ?x; gis:y ?y ] . }"


    bindings = ""
    service = ""
    serviceClient = rospy.ServiceProxy('/husky0/kDBServerNodelet/sparql_query', QueryDatabase)
    plan = serviceClient([graphname], "json", query_all, bindings, service)

    success = plan.success
    result = plan.result

    result_data = json.loads(result)

    rospy.init_node('visualise_semantic_objects', anonymous=False)

    display_marker_pub = rospy.Publisher('semantic_sensor_visualisation',
    visualization_msgs.msg.MarkerArray, queue_size=10, latch = True)
    marker = visualization_msgs.msg.Marker()

    i = 0
    for row in result_data["results"]["bindings"]:

        marker.id = 1000 + i

        marker.header.frame_id = "odom"
        marker.type   = visualization_msgs.msg.Marker.CUBE_LIST
        marker.action = 0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0

        x = row['x']['value']
        y = row['y']['value']
        klass = row['class']['value']


        marker.points.append(geometry_msgs.msg.Point(float(x), float(y), 0))

        if klass == 'human':
            marker.colors.append(std_msgs.msg.ColorRGBA(1.0, 0.5, 0.5, 1.0))
        else:
            marker.colors.append(std_msgs.msg.ColorRGBA(0.5, 1.0, 0.5, 1.0))

        i = i + 1

    display_marker_pub.publish(visualization_msgs.msg.MarkerArray([marker]))

if __name__ == '__main__':
    visualize_db("semanticobject")
    print("RUNNING")
    rospy.spin()
