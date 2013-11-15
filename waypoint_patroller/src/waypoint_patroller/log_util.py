"""
Provides utility functions for interpretting logged patrol runs
"""
import rospy
import strands_datacentre.util as dc_util

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
        self.start_mileage = 0
        self.bumper_hits=[]
        self.finish_time = None
        self.start_mileage = 0
        self.finish_mileage = 0
        self.stamped_mileage=[]
        self.stamped_battery=[]
        self.stamped_charges=[]

    def fill_from_log(self, events):
        """ Fill this episode stat from a pymongo iterator of documents, all assumed to be
        event logs for this episode """
        for event in events.sort('stamp.secs'):
            if not event['episode_name']==self.name:
                raise Exception("Trying to fill in an episode object with wrong events.")
            event_type = event['event_type']

            if event_type == "episode start":
                self.start_time = stamp_to_datetime(event['stamp'])
                self.start_mileage = event['mileage']

            if self.start_time is None:
                # Problem - should never get to this as the first event should be a start event..
                continue
                #raise ()

            self.stamped_mileage.append([ event['mileage'] - self.start_mileage,
                                          stamp_to_datetime(event['stamp']) - self.start_time ])
            self.stamped_battery.append([ event['battery level'],
                                          stamp_to_datetime(event['stamp']) - self.start_time ])

            if event_type == "bumper pressed":
                self.bumper_hits.append(stamp_to_datetime(event['stamp']) - self.start_time )

            if event_type == "charge started":
                self.bumper_hits.append(stamp_to_datetime(event['stamp']) - self.start_time )

            if event_type == "episode finish":
                self.finish_time = stamp_to_datetime(event['stamp'])
                self.finish_mileage = event['mileage']

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
        complete["finish_time"] = str(self.finish_time)
        complete["start_mileage"] = self.start_mileage
        complete["finish_mileage"] = self.finish_mileage
        complete["stamped_mileage"]=[[i[0], str(i[1])] for i in self.stamped_mileage ]
        complete["stamped_battery"]=[[i[0], str(i[1])] for i in self.stamped_battery ]
        complete["stamped_charges"]=[[i[0], str(i[1])] for i in self.stamped_charges ]
        
        return json.dumps(complete)

    def get_json_summary(self):
        summary = {}
        summary['episode_name']=self.name
        summary['date']=str(self.start_time)
        summary['run_duration']=str(self.stamped_mileage[-1][1])
        summary['distance']=self.stamped_mileage[-1][0]
        summary['bump_recoveries']=len(self.bumper_hits)
        summary['charge_cycles']=len(self.stamped_charges)
        return json.dumps(summary)


                

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
        

