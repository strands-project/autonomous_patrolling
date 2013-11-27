#!/usr/bin/env python

from optparse import OptionParser
import pymongo
import datetime

def create_stats(episode_name=None, dc_host="localhost", dc_port=62345, t_minus_s=1200):
    """ Returns a list of event tuples """

    client = pymongo.MongoClient(dc_host, dc_port)
    
    db = client["autonomous_patrolling"]["episodes"]

    episodes = [ i for i in db.distinct("episode_name")]
    if episode_name is None:
        episode_name = episodes[-1]        
    elif episode_name not in episodes:
        raise Exception("The suplied episode is not in the datacentre.")

    print "Episode Name: ", episode_name
    start = db.find({'episode_name':episode_name, 'event_type':'episode start'})[0]
    print "Episode Date: ", datetime.datetime.fromtimestamp(start['stamp']['secs'])
    number_waypoint_success = db.find({'episode_name':episode_name, 'event_type':'success waypoint'}).count()
    number_waypoint_fail = db.find({'episode_name':episode_name, 'event_type':'failed waypoint'}).count()
    print "Number of waypoints: ", number_waypoint_success, " / ", number_waypoint_fail+number_waypoint_success
    print "="*35
    print "The last ", t_minus_s, "seconds of events:"
    print

    now = datetime.datetime.now()
    last = None
    for i in db.find({'episode_name':episode_name}).sort('stamp.secs', pymongo.DESCENDING):
        event_type = i['event_type']
        if last is None:
            last = datetime.datetime.fromtimestamp(i['stamp']['secs'])
        time = datetime.datetime.fromtimestamp(i['stamp']['secs'])
        if last - time < datetime.timedelta(seconds=t_minus_s):
            #print it
            print time, "\tbattery=",i['battery level'],"\tmileage=",i['mileage'],"\t",
            if event_type == "start waypoint":
                print "starting waypoint [",i['waypoint'],"]",
            elif event_type=="failed waypoint":
                print "waypoint failed.",
            elif event_type=="success waypoint":
                print "waypoint reached.",
            elif event_type=="helped":
                print "Helped, reason=", i['type'] ,
            else:
                print event_type,
            print
                
    
if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("-d","--datacentre-host",dest="datacentre", default="localhost",
                      help="the machine that the datacentre(mongodb) is on")

    parser.add_option("-p","--datacentre-port",dest="datacentre_port",  type="int", default="62345",
                      help="the port that the datacentre(mongodb) is on")

    parser.add_option("-e", "--episode", dest="episode_name", default=None,
                      help="The name of the episode to show stats for")
    
    parser.add_option("-t","--history",dest="t_minus",  type="int", default="1200",
                      help="How many seconds of history")

    (options,args) = parser.parse_args()

    create_stats(options.episode_name, options.datacentre, options.datacentre_port, options.t_minus)
