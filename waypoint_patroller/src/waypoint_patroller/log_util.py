"""
Provides utility functions for interpretting logged patrol runs
"""
import rospy
import ros_datacentre.util as dc_util

got_pymongo = dc_util.check_for_pymongo()
if not got_pymongo:
    raise Exception("Can't find pymongo library")
import pymongo

import datetime
import json

def stamp_to_datetime(stamp):
    """
    Takes a datacentre timestamp (stamp.secs, stamp.nsecs) and returns a python datatime object
    """
    return datetime.datetime.fromtimestamp(stamp['secs'] + stamp['nasecs']/1e9)


class Episode(object):
    """
    Information related to a single patrol run
    """
    
    def __init__(self, name):
        self.name = name
        self._populated = False

        self.start_time = None
        self.latest_event_time = None
        self.start_mileage = 0
        self.bumper_hits=[]
        self.navigation_fails=[]
        self.finish_time = None
        self.start_mileage = 0
        self.finish_mileage = 0
        self.stamped_mileage=[]
        self.stamped_battery=[]
        self.stamped_charges=[]
        self.waypoints_stamps=[]
        self.active_point_name = ""
        self.bump_help_count = 0
        self.navigation_help_count = 0
        

    def fill_from_log(self, events):
        """ Fill this episode stat from a pymongo iterator of documents, all assumed to be
        event logs for this episode """
        for event in events.sort('stamp.secs'):
            if not event['episode_name']==self.name:
                raise Exception("Trying to fill in an episode object with wrong events.")
            event_type = event['event_type']

            if event_type == "episode start":
                self.start_time = stamp_to_datetime(event['stamp'])
                self.start_mileage = event['odom_mileage']

            if self.start_time is None:
                # Problem - should never get to this as the first event should be a start event..
                continue
                #raise ()

            self.stamped_mileage.append([ event['odom_mileage'] - self.start_mileage,
                                          stamp_to_datetime(event['stamp']) - self.start_time ])
            self.stamped_battery.append([ event['battery level'],
                                          stamp_to_datetime(event['stamp']) - self.start_time ])
            self.latest_event_time = stamp_to_datetime(event['stamp'])

            if event_type ==  "heartbeat":
                continue
            
            if event_type == "bumper pressed":
                self.bumper_hits.append(stamp_to_datetime(event['stamp']) - self.start_time )

            if event_type == "navigation recovery":
                self.navigation_fails.append(stamp_to_datetime(event['stamp']) - self.start_time )

            if event_type == "charging finished":
                self.stamped_charges.append(stamp_to_datetime(event['stamp']) - self.start_time )

            if event_type == "episode finish":
                self.finish_time = stamp_to_datetime(event['stamp'])
                self.finish_mileage = event['odom_mileage']

            if event_type == "start waypoint":
                self.active_point_name = event["waypoint"]

            if event_type == "success waypoint":
                self.waypoints_stamps.append([ self.active_point_name, True,
                                          stamp_to_datetime(event['stamp']) - self.start_time ])
            
            if event_type == "failed waypoint":
                self.waypoints_stamps.append([ self.active_point_name, False,
                                          stamp_to_datetime(event['stamp']) - self.start_time ])
            
            if event_type == "helped":
                if event['type'] == "navigation":
                    self.navigation_help_count += 1
                elif event['type'] == "bumper":
                    self.bump_help_count += 1

        self._populated = True

    def __str__(self):
        """
        For debug printing
        """
        return """Episode Name: %s
Date: %s
Run duration: %s
Distance covered: %s
Bumper recoveries: %d
Charge cycles: %d
""" % (self.name, self.start_time,self.stamped_mileage[-1][1],
       self.stamped_mileage[-1][0],len(self.bumper_hits),len(self.stamped_charges))

    def get_json_complete(self):
        complete = {}
        complete["start_time"] = str(self.start_time)
        complete["start_mileage"] = self.start_mileage
        complete["bumper_hits"]=[str(i) for i in self.bumper_hits]
        complete["navigation_fails"]=[str(i) for i in self.navigation_fails]
        complete["finish_time"] = str(self.finish_time)
        complete["start_mileage"] = self.start_mileage
        complete["finish_mileage"] = self.finish_mileage
        complete["stamped_mileage"]=[[i[0], str(i[1])] for i in self.stamped_mileage ]
        complete["stamped_battery"]=[[i[0], str(i[1])] for i in self.stamped_battery ]
        complete["stamped_charges"]=[[i[0], str(i[1])] for i in self.stamped_charges ]
        complete["stamped_waypoints"] = [[i[0], i[1], str(i[2])] for i in self.waypoints_stamps]
        complete["current_waypoint"]=self.active_point_name
        complete['last_event_time']=str(self.latest_event_time)
        complete['bump_help_count']= self.bump_help_count
        complete['navigation_help_count']=self.navigation_help_count
        
        return json.dumps(complete)

    def get_summary(self):
        summary = {}
        summary['episode_name']=self.name
        summary['date']=str(self.start_time)
        summary['run_duration']=self.stamped_mileage[-1][1].total_seconds()
        summary['distance']=self.stamped_mileage[-1][0]
        summary['bump_recoveries']=len(self.bumper_hits)
        summary['navigation_recoveries']=len(self.navigation_fails)
        summary['charge_cycles']=len(self.stamped_charges)
        summary['successful_waypoints']=sum([1 if i[1] else 0 for i in self.waypoints_stamps ])
        summary['failed_waypoints']=sum([1 if not i[1] else 0 for i in self.waypoints_stamps ])
        summary['active_waypoint']=self.active_point_name
        summary['last_event_time']=str(self.latest_event_time)
        summary['bump_help_count']= self.bump_help_count
        summary['navigation_help_count']=self.navigation_help_count
        return summary
        
    def get_json_summary(self):
        summary = self.get_summary()
        return json.dumps([summary])


                

class StatGenerator(object):
    """
    Connects to MongoDB and pulls out statistics for runs
    """
    
    def __init__(self, db_server='localhost', db_port=62345):
        """
        Arguments:
        - `db_server`:
        - `db_port`:
        """
        self._db_server = db_server
        self._db_port = db_port
        
        self._client = pymongo.MongoClient(self._db_server, self._db_port)
        self._collection = self._client.autonomous_patrolling.episodes
        self._episodes={}

    def _update_episodes(self):
        self._episodes={}
        for i in self._collection.find().distinct("episode_name"):
            self._episodes[i]=Episode(i)

    def _update_episode(self, episode_name):
        """
        Calculate statistics for the given episode
        Arguments:
        - `episode_name`:
        """
        episode = Episode(episode_name)
        episode.fill_from_log(self._collection.find({'episode_name':episode_name}))
        self._episodes[episode_name]=episode
        
        
    def get_episode_names(self):
        """
        Returns a list of the episode names        
        """
        names = [ i for i in self._collection.find().distinct("episode_name")]
        return names

    def get_episode_names_since(self, start_date, end_date=None):
        """
        Gets a list of episodes that started since the start_date, started before end_date
        """
        candidates = self.get_episode_names()
        episodes=[]
        for i in candidates:
            start = self._collection.find_one({"episode_name":i,"event_type":"episode start"})
            time = stamp_to_datetime(start["stamp"])
            if time > start_date and ( end_date is None or time < end_date):
                episodes.append(i)
        return episodes
            

    def get_episode(self, episode_name):
        """
        Return the Episode
        """
        if episode_name not in self.get_episode_names():
            raise Exception("Unknown episode name")
        refresh = (self.get_latest_run_name()==episode_name)
        if refresh or not self._episodes.has_key(episode_name):
            self._update_episode(episode_name)
        ep = self._episodes[episode_name]
        if not ep._populated:
            self._update_episode(episode_name)
            
        return self._episodes[episode_name]

    def get_latest_run_name(self):
        """
        Return the name of the run that has the most recent time stamp
        """
        name = self._collection.find().sort('stamp.secs').distinct("episode_name")[-1]
        return name
        


    
