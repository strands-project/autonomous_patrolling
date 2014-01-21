#!/usr/bin/env python

class topological_node(object):
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
        

class topological_nav_map(object):
    def __init__(self,file_name):
        self._load_map_file(filename)

    def _load_map_file(self,file_name) :
        print "openning %s" %inputfile 
        fin = open(inputfile, 'r')
        print "Done"
        line = fin.readline()
        node=topological_node("Empty")
        self.lnodes=[node]
        while line:
            if line.startswith('node:') :
                line = fin.readline()
                name = line.strip('\t')
                name = name.strip('\n')
                node=topological_node(name)
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
                self.lnodes.append(node)
        fin.close()
        self.lnodes.pop(0)
        for i in self.lnodes:
            print i.node_name
        #return lnodes
    