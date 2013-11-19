#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import subprocess
import sys
from optparse import OptionParser
from time import sleep
import waypoint_patroller.log_util

def create_sumary_file():
    gen = waypoint_patroller.log_util.StatGenerator()
    summary = gen.get_episode(gen.get_latest_run_name()).get_json_summary()
    with  open("/tmp/patrol_run.json", "w") as f:
        f.write(summary)

def upload_summary_scp(upload_path, server, user, password):
    call = subprocess.call(["sshpass", "-p", password, "scp", "/tmp/patrol_run.json", "%s@%s:%s"%(user,server,upload_path) ])
    if call != 0:
        raise Exception("Failed to upload summary. Bad call to sshpass...scp")
    
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("-s","--server",dest="hostname",
                      help="the ssh host to copy the file to")
    parser.add_option("-u","--username",dest="username",
                      help="username for account on ssh server")
    parser.add_option("-p","--password",dest="password", default="nopass",
                      help="the users password")
    parser.add_option("-f","--filepath",dest="path",
                      help="the location to place the file on the server.")
    parser.add_option("-t","--time-between",dest="timeout", type="int", default=300,
                      help="the time in seconds to pause between uploads to server. default = 300 seconds")

    (options,args) = parser.parse_args()

    while True:
        create_sumary_file()
        upload_summary_scp(options.path, options.hostname, options.username, options.password)
        print "File uploaded."
        sleep(options.timeout)
