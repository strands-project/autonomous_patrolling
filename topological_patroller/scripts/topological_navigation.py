#!/usr/bin/env python

import rospy
import actionlib
import topological_patroller.msg
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
import scitos_apps_msgs.msg

class Topo_node(object):
    def __init__(self,node_name):
        self.node_name=node_name
        self.expanded=False
        self.father='none'

    def _insert_waypoint(self, waypoint):
        self.waypoint=waypoint

    def _insert_edges(self, edges):
        self.edges=edges
        
    def _get_Children(self) :
        self.expanded=True
        a=['none']
        for k in self.edges :
            a.append(k['node'])
        a.pop(0)
        return a
        
    def _get_action(self, node) :
        for k in self.edges :
            if k['node'] == node :
                return k['action']
    
    def _set_Father(self,father) :
        self.father=father

def loadMap(inputfile):
    fin = open(inputfile, 'r')
    line = fin.readline()
    node=Topo_node("Empty")
    lnodes=[node]
    while line:
        if line.startswith('node:') :
            line = fin.readline()
            name = line.strip('\t')
            name = name.strip('\n')
            node=Topo_node(name)
            line = fin.readline()
            if line.startswith('\t') :
                if line.startswith('\twaypoint:') :
                    line = fin.readline()
                    ways = line.strip('\t')
                    ways = ways.strip('\n')
                    node._insert_waypoint(ways)
            line = fin.readline()
            if line.startswith('\t') :
                if line.startswith('\tedges:') :
                    edge = {'node':"empty", 'action':"move_base"}
                    edges=[edge]
                    line = fin.readline()
                    while line and not(line.startswith('node:')) :
                        info= line.strip('\t')
                        inf = info.split(',',2)
                        edge = {'node':inf[0].strip(), 'action':inf[1].strip()}
                        edges.append(edge)
                        line = fin.readline()
                    edges.pop(0)
                    node._insert_edges(edges)
            lnodes.append(node)
    fin.close()
    lnodes.pop(0)
    return lnodes

def findInList(name,List):
    found = name in List
    if found :
        pos = List.index(name)
    else :
        pos = -1
    return pos


def update_to_expand(to_expand, new_nodes, maptree, father) :
    for i in new_nodes :
        found_el=False
        for j in to_expand :
            if i == j.node_name :
                found_el=True
        if not found_el :
            for k in maptree :
                if i == k.node_name :
                    new_el=k
                    new_el._set_Father(father)
                    to_expand.append(new_el)
    return to_expand

def get_node(name, maptree) :
    for i in maptree :
        if i.node_name == name :
            return i
    return None

class TopologicalNavServer(object):
    _feedback = topological_patroller.msg.GotoNodeFeedback()
    _result   = topological_patroller.msg.GotoNodeResult()

    def __init__(self, name, filename):
        self.cancelled = False
        self._action_name = name
        
        self.lnodes = loadMap(filename)
            
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, topological_patroller.msg.GotoNodeAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.loginfo("Creating base movement client.")
        self.baseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.baseClient.wait_for_server()
        rospy.loginfo("Base client initialized")

        self.rampClient = actionlib.SimpleActionClient('rampClimbingServer', scitos_apps_msgs.msg.RampClimbingAction)
        #print "wait for Server"
        self.rampClient.wait_for_server()
        rospy.loginfo("Ramp client initialized")

        rospy.loginfo("Ready to Rumble ...")
        rospy.spin()

    def executeCallback(self, goal):
        self.cancelled = False
        self._feedback.route = 'Starting...'
        self._as.publish_feedback(self._feedback)
        rospy.loginfo('%s: Navigating From %s to %s' %(self._action_name, goal.origin, goal.target))
        Onode = get_node(goal.origin, self.lnodes)
        Gnode = get_node(goal.target, self.lnodes)
        if (Gnode is not None) and (Onode is not None) :
            exp_index=0
            to_expand=[Onode]
            to_expand[exp_index]._set_Father('none')
            children=to_expand[exp_index]._get_Children()
            not_goal=True
            while not_goal :
                pos=findInList(goal.target, children)
                if pos>=0 :
                    print "Goal found in Pos %d" %pos
                    not_goal=False
                else :
                    print "Goal NOT found"
                    update_to_expand(to_expand, children, self.lnodes, to_expand[exp_index].node_name)
                    exp_index=exp_index+1
                    print "nodos para expandir %d:" %len(to_expand)
                    for m in to_expand :
                        print m.node_name
                    print "expandiendo nodo %d: (%s)" %(exp_index,to_expand[exp_index].node_name)
                    if exp_index >= len(to_expand) :
                        not_goal=False
                    children=to_expand[exp_index]._get_Children()
                    print "nodos en la lista:"
                    print children
        
            print "fixing Father %s for goal %s" %(to_expand[exp_index].node_name,Gnode.node_name)
            Gnode._set_Father(to_expand[exp_index].node_name)
            print "Father for Gnode %s" %(Gnode.father)
            route=[Gnode]
            #del route[:]
            print "Ruta actual %d" %len(route)
            rindex=0
            print route[rindex].father
            while route[rindex].father is not 'none' :
                nwnode = get_node(route[rindex].father, to_expand)
                route.append(nwnode)
                rindex=rindex+1
            
            route.reverse()
            result=self.followRoute(route)

        else :
            rospy.loginfo("Either Target or Origin Nodes were not found on Map")  
            result=False #self._send_tweet(goal.text)
            
        if not self.cancelled :
            self._result.success = result
            self._feedback.route = goal.target
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)
        del to_expand[:]
        

    def followRoute(self, route):
        rospy.loginfo("%d Nodes on route" %len(route))
        movegoal = MoveBaseGoal()
        rindex=0
        nav_ok=True
        while rindex < (len(route)-1) and not self.cancelled and nav_ok :
            a = route[rindex]._get_action(route[rindex+1].node_name)
            print "From %s do %s to %s" %(route[rindex].node_name,a,route[rindex+1].node_name)

            if a is 'move_base' :
                print "move_base to:" 
                inf = route[rindex+1].waypoint.split(',',7)
                print inf
                movegoal.target_pose.header.frame_id = "map"
                movegoal.target_pose.header.stamp = rospy.get_rostime()
                movegoal.target_pose.pose.position.x = float(inf[0])
                movegoal.target_pose.pose.position.y = float(inf[1])
                movegoal.target_pose.pose.orientation.x = 0
                movegoal.target_pose.pose.orientation.y = 0
                movegoal.target_pose.pose.orientation.z = float(inf[5])
                movegoal.target_pose.pose.orientation.w = float(inf[6])
                self.baseClient.cancel_all_goals()
                rospy.sleep(rospy.Duration.from_sec(1))
                print movegoal
                self.baseClient.send_goal(movegoal)
                self.baseClient.wait_for_result()
                if self.baseClient.get_state() != GoalStatus.SUCCEEDED:
                    nav_ok=False
                rospy.sleep(rospy.Duration.from_sec(0.3))
            elif a is 'ramp_climbing' :
                print "ramp_climbing"
                rampgoal = scitos_apps_msgs.msg.RampClimbingGoal()
                rampgoal.Timeout = 1000
                print "sending goal"
                print rampgoal
                self.rampClient.send_goal(rampgoal)
                self.rampClient.wait_for_result()
                
        rindex=rindex+1
        result=nav_ok
        return result

    def preemptCallback(self):
        self.cancelled = True
        self._result.success = False
        self._as.set_preempted(self._result)
        

if __name__ == '__main__':
    filename=str(sys.argv[1])
    rospy.init_node('topological_navigation')
    server = TopologicalNavServer(rospy.get_name(),filename)
