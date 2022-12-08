#!/usr/bin/env python
import clips
import time, threading, logging, yaml, sys, os
from io import StringIO

import std_msgs.msg

from ros_clips.msg import StringArray
from ros_clips.srv import RunPlanning, GetPrintMessage

import rospy
import rospkg

#Initialize CLIPS Environment
env = clips.Environment()

#Start IO String
log_stream = StringIO()

#Constants
defaultTimeout = 2000
defaultAttempts = 1
logLevel = 1

run_steps = None

#Thread Locker
_clipsLock = threading.Lock()

#Defs
def resetLogStream():
    log_stream.truncate(0)
    log_stream.seek(0)

def getLogStream():
    _log = []
    _log = log_stream.getvalue().splitlines()
    for i in range(len(_log)):
        _log[i] = _log[i].rstrip('. ')
    print(_log)
    return _log


#Topics' callbacks
def callbackCLIPSReset(data):
    print ('\nFacts are being reset.')
    _clipsLock.acquire()
    
    env.reset()
    resetLogStream()
    
    _clipsLock.release()


def callbackCLIPSClear(data):
    print ("\nEnviroment is being cleared.")
    _clipsLock.acquire()
        
    env.clear()
    resetLogStream()
    
    _clipsLock.release()


def callbackCLIPSLoadFile(data):
    print ('\nLoading File...')
    #filepath = data.data
    filepath = os.path.abspath(os.path.expanduser(os.path.expandvars(data.data)))
    if filepath[-3:] == 'clp':
        _clipsLock.acquire()
        env.batch_star(filepath)
        print ('File Loaded')
        
        env.reset()
        resetLogStream()
        print ('Facts were reset')
        
        _clipsLock.release()
        return
    
    path = os.path.dirname(os.path.abspath(filepath))
    f = open(filepath, 'r')
    line = f.readline()
    
    _clipsLock.acquire()
    while line:
        env.batch_star((path + os.sep + line).strip())
        line = f.readline()
    f.close()
    
    print ('Files Loaded')
    
    env.reset()
    resetLogStream()
    print ('Facts were reset')
    
    _clipsLock.release()


#####
def callbackBuildRule(data):
    print ('\nBuilding Rule: ' + data.data)
    _clipsLock.acquire()
    
    env.build(data.data)
    
    _clipsLock.release()

def callbackBuildTemplate(data):
    print ('\nBuilding Template: ' + data.data)
    _clipsLock.acquire()
    
    env.build(data.data)
    
    _clipsLock.release()

def callbackBuildFacts(data):
    print ('\nBuilding Facts: ' + data.data)
    _clipsLock.acquire()
    
    env.build(data.data)
    
    _clipsLock.release()

def callbackLoadFact(data):
    print ('\nLoading Fact: ' + data.data)
    _clipsLock.acquire()
    
    env.load_facts(data.data)
    
    _clipsLock.release()


#Services' callbacks
def callbackRunPlanningService(req):

    print ("\nPlanning and Running:")
    _clipsLock.acquire()
    
    limit = req.steps
    if limit is not None and int(limit) > 0:
        limit = int(limit)+1
    else:
        limit = None
    
    resetLogStream()
    env.run(limit)
    _p = getLogStream()
    plan = StringArray(_p)
    
    _clipsLock.release()
    return plan 

def callbackGetFactsService(req):

    print ("\nGetting Facts:")
    _clipsLock.acquire()

    facts = []
    for _fact in env.facts():
        f = str(_fact.index) + " " + str(_fact)
        f = f.lstrip()
        #_id,_f = f.split(maxsplit=1)
        if (len(f)>0):
            facts.append(f)

    print(facts)
    data = StringArray(facts)
    
    _clipsLock.release()
    return data 

def callbackGetRulesService(req):

    print ("\nGetting Rules:")
    _clipsLock.acquire()

    rules = []
    for _rule in env.rules():
        r = str(_rule)
        r = r.lstrip()
        if (len(r)>0):
            rules.append(r)

    print(rules)
    data = StringArray(rules)
    
    _clipsLock.release()
    return data 

def callbackGetTemplatesService(req):

    print ("\nGetting Templates:")
    _clipsLock.acquire()

    templates = []
    for _templates in env.templates():
        t = str(_templates)
        t = t.lstrip()
        if (len(t)>0):
            templates.append(t)

    print(templates)
    data = StringArray(templates)
    
    _clipsLock.release()
    return data 

#Main
def main():
    rospy.init_node('clips_ros_bridge')
    
    #Start IO Router
    logger = logging.getLogger()
    streamHandler = logging.StreamHandler(log_stream)
    #streamHandler = logging.StreamHandler(sys.stdout)
    formatter = logging.Formatter('%(message)s')
    streamHandler.setFormatter(formatter)
    logger.addHandler(streamHandler)
    
    router = clips.LoggingRouter()
    env.add_router(router)
    
    #Start ROS Subscribers
    rospy.Subscriber("/ros_clips/clipspy_reset",std_msgs.msg.Empty, callbackCLIPSReset)
    rospy.Subscriber("/ros_clips/clipspy_clear",std_msgs.msg.Empty, callbackCLIPSClear)
    rospy.Subscriber("/ros_clips/clipspy_load_file",std_msgs.msg.String, callbackCLIPSLoadFile)
    
    rospy.Subscriber("/ros_clips/clipspy_build_rule",std_msgs.msg.String, callbackBuildRule)
    rospy.Subscriber("/ros_clips/clipspy_build_template",std_msgs.msg.String, callbackBuildTemplate)
    rospy.Subscriber("/ros_clips/clipspy_build_facts",std_msgs.msg.String, callbackBuildFacts)
    rospy.Subscriber("/ros_clips/clipspy_load_fact",std_msgs.msg.String, callbackLoadFact)
    
    #Start ROS Services
    rospy.Service("/ros_clips/run_planning", RunPlanning, callbackRunPlanningService)
    rospy.Service("/ros_clips/get_facts", GetPrintMessage, callbackGetFactsService)
    rospy.Service("/ros_clips/get_rules", GetPrintMessage, callbackGetRulesService)
    rospy.Service("/ros_clips/get_templates", GetPrintMessage, callbackGetTemplatesService)
    
    rospy.loginfo("ROS-CLIPS Initialized")
    
    #Infinite loop
    rospy.spin()


if __name__ == "__main__":
    main()
