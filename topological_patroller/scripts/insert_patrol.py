#!/usr/bin/env python

from time import sleep
import rospy
import smach
import smach_ros
import datetime
import json
import sys

#from topological_patroller.patrol import *
from topological_patroller.msg import *
from ros_datacentre.message_store import MessageStoreProxy



def loadTask(inputfile, task_name):
    print "openning %s" %inputfile 
    fin = open(inputfile, 'r')

    patrol = topological_patroller.msg.PatrolTask()
    patrol.task_name = task_name
    line = fin.readline()

    while line:
        if line.startswith('CheckPoint:') :
            line = fin.readline()
            line = line.strip('\t')
            line = line.strip('\n')
            task = topological_patroller.msg.CheckPointTask()
            task.waypoint = line
            line = fin.readline()
            
            if line.startswith('\tAction:') :
                line = fin.readline()
                while line and not(line.startswith('CheckPoint:')) :
                    info= line.strip('\t')
                    info2= info.strip('\n')
                    inf = info2.split(';',2)
                    taction = topological_patroller.msg.CheckPointAction()
                    taction.name = inf[0].strip()
                    starg = inf[1].strip(' ')
                    actargs = starg.split(',')
                    for j in actargs :
                        taction.args.append(j)
                    task.action.append(taction)
                    line = fin.readline()
            patrol.patrol.append(task)
        else:
            line = fin.readline()
    fin.close()
    return patrol



if __name__ == '__main__':
    
    rospy.init_node('task_insertion')
    patrol_points = loadTask(sys.argv[1], sys.argv[2])
    print patrol_points
    meta = {}
    meta["type"] = "task_definition"
    meta["task"] = sys.argv[2]
        
    msg_store = MessageStoreProxy(collection='patrol_tasks')
    msg_store.insert(patrol_points,meta)
    
