#!/usr/bin/env python
#import roslib; roslib.load_manifest("waypoint_visualiser")
import rospy
import sys

from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Pose

from ros_datacentre.message_store import MessageStoreProxy
import csv


def way_points_file_to_datacentre(filename, dataset_name, map_name):
    msg_store = MessageStoreProxy()

    points=[]
    with open(filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            current_row=[]
            for element in row:
                current_row.append(float(element))
            points.append(current_row)


    for i,point in enumerate(points):
        meta = {}
        meta["map"] = map_name
        meta["pointset"] = dataset_name

        if i == 0:
            meta["name"] = "charging_point"
        else:
            meta["name"]= "Point%d"%i

        p = Pose()
        p.position.x=point[0]
        p.position.y=point[1]
        p.position.z=point[2]
        p.orientation.x=point[3]
        p.orientation.y=point[4]
        p.orientation.z=point[5]
        p.orientation.w=point[6]

        msg_store.insert(p,meta)

    # this just pulls them back out again 
    # query_meta = {}
    # query_meta["map"] = map_name
    # query_meta["pointset"] = dataset_name
    # print msg_store.query(Pose._type, {}, query_meta)

if __name__=="__main__":
    if len(sys.argv)!=4:
        print "Usage: rosrun waypoint_recorder insert_in_db.py 'csv_file' 'dataset_name' 'map_name'"
        sys.exit(1)
    way_points_file_to_datacentre(sys.argv[1],sys.argv[2],sys.argv[3])
