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


def drive_to_humans(graphname):
    query_all = "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>\
    PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>\
    \
    SELECT ?x ?y ?id WHERE { ?id a <human> ;\
                  properties:location [ gis:x ?x; gis:y ?y ] . }"

    bindings = ""
    service = ""
    serviceClient = rospy.ServiceProxy('/husky0/kDBServerNodelet/sparql_query', QueryDatabase)
    plan = serviceClient([graphname], "json", query_all, bindings, service)

    success = plan.success
    result = plan.result

    result_data = json.loads(result)

    plan = {'children':[], "common_params": {}, "name": "seq", "params":{} }

    for row in result_data["results"]["bindings"]:
        x = row['x']['value']
        y = row['y']['value']
        use_mp = False
        plan["children"].append(add_child(float(x),float(y),use_mp))

    json_plan = json.dumps(plan, indent = 4)
    with open("drive_to_humans.json", "w") as outfile:
        outfile.write(json_plan)


def add_child(x,y, use_mp):
    child = {"children": [], "common_params": {}, "name": "drive-to", "params": {
    "p" : {
        "rostype" : "Point",
        "x": x,
        "y": y,
        "z": 0
        },
    "use-motion-planner": use_mp
    }}
    return child


if __name__ == '__main__':
    drive_to_humans("semanticobject")
    print("RUNNING")
