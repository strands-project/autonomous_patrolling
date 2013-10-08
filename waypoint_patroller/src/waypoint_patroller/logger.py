import rospy
import strands_datacentre.util as dc_util

from std_msgs.msg import Float32
from scitos_msgs.msg import BatteryState

got_pymongo = dc_util.check_for_pymongo()
if not got_pymongo:
    raise Exception("Can't find pymongo library")
import pymongo

"""
A base class for all states that are going to have logging capabilities. It
stores and a reference to the logger and shares among with children.
"""
class Loggable(object):
    def _ensure_logger(self):
        if not hasattr(self, "_logger"):
            self._logger=Logger()
    
    def set_logger(self,logger):
        assert isinstance(logger, Logger)
        self._logger=logger
        for a in dir(self):
            attr = getattr(self, a)
            if isinstance(attr, Loggable):
                rospy.logerr("Passing on logger to " + a)
                attr.set_logger(self._logger)
                        
    def get_logger(self):
        self._ensure_logger()
        return self._logger
    
"""
Base class for loggers, simply the non-logger: it silently logs nothing
"""
class Logger(object):
    def __getattr__(self, name):
        isinstance(name, str)
        if name.startswith("log_"):
            return lambda *args: False
        else:
            raise AttributeError("Logger has no attrribute %s"%name)

"""
Logging class for entering run data into the strands_datacentre.
This class should be created once and each state that wants to log should
hold a reference to it.
"""
class PatrollLogger(Logger):
    def __init__(self, name="autonomous_patroller"):
        self._active_episode = False
        got_datacentre = dc_util.wait_for_mongo()
        if not got_datacentre:
            raise Exception("strands_datacentres does not appear to be running")
        self._mongo = pymongo.MongoClient(rospy.get_param("datacentre_host",
                                                          "localhost"),
                                          int(rospy.get_param("datacentre_port")))
        self._db = self._mongo[name].episodes
        
        self._mileage_sub = rospy.Subscriber('/mileage', Float32,
                                             self._mileage_cb)
        self._batterystate_sub = rospy.Subscriber('/battery_state', BatteryState,
                                             self._batterystate_cb)
        self._mileage = 0
        self._battery = 100
        

    def _mileage_cb(self, msg):
        self._mileage = msg.data
        
    def _batterystate_cb(self, msg):
        self._battery = msg.lifePercent

    """ put a ros timestamp on a document """
    def _stamp(self, doc, now=None):
        if now is None:
            now =  rospy.get_rostime()
        doc["stamp"] = {"secs": now.secs, "nasecs": now.nsecs,}
        return doc

    def _record(self, doc):
        if self._active_episode:
            doc['episode_name'] = self._episode_name
            doc['mileage'] = self._mileage
            doc['battery level'] = self._battery
            self._db.insert(self._stamp(doc))
        

    def log_start_episode(self, pt_set_name):
        #rospy.logerr("[NotImp] Episode started!!!!!!")
        # create a name for the episode
        t = rospy.get_rostime()
        self._episode_name = '%d.%d' % (t.secs, t.nsecs)
        self._active_episode = True
        
        self._record({'event_type': 'episode start',
                      'waypoint_set': pt_set_name, })
            
    def log_finish_episode(self):
        self._record({'event_type': 'episode finish', })
        self._active_episode = False
    
    def log_waypoint_visit(self, point_name):
        self._record({'event_type': 'start waypoint',
                      'waypoint': point_name, })
                
    def log_waypoint_success(self, waypoint):
        self._record({'event_type': 'success waypoint', })
    
    def log_waypoint_fail(self, waypoint):
        self._record({'event_type': 'failed waypoint', })
    
    def log_bump(self):
        self._record({'event_type': 'bumper pressed', })

    def log_bump_count(self,number):
        self._record({'event_type': 'bump recovered',
                      'number of resets':number})

    def log_intervention_alarm(self):
        pass
    
    def log_navigation_recovery_attempt(self, success, attempts):
        self._record({'event_type': 'navigation recovery',
                      'success': success,
                      'attempts': attempts,})
    
    def log_mileage(self, mileage):
        pass
    
    def log_charging_start(self):
        self._record({'event_type': 'charging started', })
    
    def log_charging_finish(self):
        self._record({'event_type': 'charging finished', })

            
    def log_carpet_stuck(self):
        self._record({'event_type': 'stuck in carpet', })
    

    
    
    
