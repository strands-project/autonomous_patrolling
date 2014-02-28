#!/usr/bin/env python
import roslib; roslib.load_manifest("waypoint_visualiser")
import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose

import pymongo
import csv

import ros_datacentre.util
from ros_datacentre.message_store import MessageStoreProxy

def way_points_file_to_datacentre(filename, dataset_name, map_name):
    host = rospy.get_param("datacentre_host")
    port = rospy.get_param("datacentre_port")
    print host, port
    client = pymongo.MongoClient(host, port)
    db=client.autonomous_patrolling
    points_db=db["waypoints"]
    
    points=[]
    with open(filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            current_row=[]
            for element in row:
                current_row.append(float(element))
            points.append(current_row)

    for i,point in enumerate(points):
#        p = {}
#        p["position"]={}
#        p["orientation"]={}
#        p["position"]["x"]=point[0]
#        p["position"]["y"]=point[1]
#        p["position"]["z"]=point[2]
#        p["orientation"]["x"]=point[3]
#        p["orientation"]["y"]=point[4]
#        p["orientation"]["z"]=point[5]
#        p["orientation"]["w"]=point[6]
#        p["header"]={}
#        p["header"]["frame_id"]="/map"
        entry={}
#        entry["pose"]=p
        entry["map"]=map_name
        entry["name"]="Point%d"%i
        entry["pointset"]=dataset_name
 #       points_db.insert(entry)

        p = Pose()
        p.position.x=point[0]
        p.position.y=point[1]
        p.position.z=point[2]
        p.orientation.x=point[3]
        p.orientation.y=point[4]
        p.orientation.z=point[5]
        p.orientation.w=point[6]

        ros_datacentre.util.store_message(points_db,p,entry)

        
class Visualiser(object):
    def __init__(self):
        rospy.init_node("waypoint_visualiser", anonymous=True)

#        way_points_file_to_datacentre(sys.argv[1],"bham_lg_2","bham_lg")

        host = rospy.get_param("datacentre_host")
        port = rospy.get_param("datacentre_port")

        self._point_set= rospy.get_param("datacentre_waypoint_set")

        self._mongo_client = pymongo.MongoClient(host,port)
        db=self._mongo_client.autonomous_patrolling
        self._points_db=db["waypoints"]

        self._marker_server = InteractiveMarkerServer(self._point_set+"_markers")
        
        self.msg_store=MessageStoreProxy()

        if  not self.waypoints_available(self._point_set):
            error_str = "Desired pointset '"+_point_set+"' not in datacentre"
            rospy.logerr(error_str)
            raise Exception(error_str)
        
        self.points = self._get_points(self._point_set)  # []
        for entry in self.points:
            rospy.loginfo("Adding marker")
            #meta, p = ros_datacentre.util.document_to_msg(entry, TYPE=Pose)
#            p=Pose()
#            p.orientation.x=entry["pose"]["orientation"]["x"]
#            p.orientation.y=entry["pose"]["orientation"]["y"]
#            p.orientation.z=entry["pose"]["orientation"]["z"]
#            p.orientation.w=entry["pose"]["orientation"]["w"]
#            p.position.x=entry["pose"]["position"]["x"]
#            p.position.y=entry["pose"]["position"]["y"]
#            p.position.z=entry["pose"]["position"]["z"]
            rospy.loginfo(entry[1])
            rospy.loginfo(self._get_point(entry[1],entry[0]))

            self._create_marker(entry[1], self._get_point(entry[1],entry[0]), entry[1])

        rospy.spin()

        
        
    """ Check whether waypoints are available in db """
    def waypoints_available(self, point_set):
      query_meta = {}
      query_meta["pointset"] = point_set
      return len(self.msg_store.query(Pose._type, {}, query_meta)) > 0

    """ Get a list of points in the given set """
    def _get_points(self, point_set):        
        query_meta = {}
        query_meta["pointset"] = point_set
        message_list = self.msg_store.query(Pose._type, {}, query_meta)

        points = []
        for [point, meta] in message_list:
            # print point
            if meta["name"] != "charging_point":
                points.append([point_set, meta["name"]])
        return points

    """ Get a given waypoint pose """
    def _get_point(self, point_name, point_set):
      query_meta = {}
      query_meta["name"] = point_name      
      query_meta["pointset"] = point_set
      return self.msg_store.query(Pose._type, {}, query_meta, True)[0]
        
        
        

    def _create_marker(self, marker_name, pose,  marker_description="waypoint marker"):
        # create an interactive marker for our server
        marker = InteractiveMarker()
        marker.header.frame_id = "/map"
        marker.name = marker_name
        marker.description = marker_description

        # the marker in the middle
        box_marker = Marker()
        box_marker.type = Marker.ARROW
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.25
        box_marker.scale.z = 0.15
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )
        marker.controls.append( box_control )

        # move x
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        #move y
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        #rotate z
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        self._marker_server.insert(marker, self._marker_feedback)
        self._marker_server.applyChanges()

        if pose is not None:
            self._marker_server.setPose( marker.name, pose )
            self._marker_server.applyChanges()



    def _marker_feedback(self, feedback):
        update={}
        p = feedback.pose.position
        q = feedback.pose.orientation
        self._points_db.update({"meta.pointset":self._point_set, "meta.name":feedback.marker_name},{"$set":{"msg":{"position":{"x":p.x,
                                                                                          "y":p.y,
                                                                                          "z":p.z},
                                                                              "orientation":{"x":q.x,
                                                                                             "y":q.y,
                                                                                             "z":q.z,
                                                                                             "w":q.w }
                                                                              }
                                                                      }
                                                              })
                                                                
        
        rospy.loginfo(feedback)                                                        
       # print feedback.marker_name + " is now at x:" + str(p.x) + ", y:" + str(p.y) + ", z:" + str(p.z)  + ", w:" + str(p.w)

        
    


if __name__=="__main__":
    visualiser = Visualiser()
