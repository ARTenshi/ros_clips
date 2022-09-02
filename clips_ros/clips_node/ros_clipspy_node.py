#!/usr/bin/env python
import clips
import time, threading, logging, yaml, sys, os
from io import StringIO

from clips_ros.msg import *
from clips_ros.srv import *
#from std_msgs.msg import Bool, String
import std_msgs.msg

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
    _log = log_stream.getvalue().splitlines()
    for i in range(len(_log)):
        _log[i] = _log[i].rstrip('. ')
    
    print(_log)


def callbackCLIPSAssertCommand(data):
    print ("\nAssert name command:" + data.name)
    _clipsLock.acquire()
    
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.name)
    
    fact = '(BB_received "{0}" {1} {2} "{3}")'.format(data.name, data.id, data.successful, data.params)
    _f = env.assert_string(fact)
    print(_f)
    
    resetLogStream()
    env.run(run_steps)
    getLogStream()
    
    _clipsLock.release()


def callbackCLIPSAssertString(data):
    print ('\nAssert Command:' + data.data)
    _clipsLock.acquire()
    
    env.assert_string(data.data)
    
    _clipsLock.release()


def callbackCLIPSEvalCommand(data):
    print ('\nSending Command:' + data.command)
    _clipsLock.acquire()
    
    env.eval(data.command)
    
    _clipsLock.release()


def callbackCLIPSEvalString(data):
    print ('\nSending Command:' + data.data)
    _clipsLock.acquire()
    
    env.eval(data.data)
    
    _clipsLock.release()


def callbackCLIPSRun(data):
    print ("\nPlanning and Running:")
    _clipsLock.acquire()
    
    steps = data.data
    if steps.isnumeric() and int(steps) > 0:
        steps = int(steps)
    else:
        steps = None
    
    resetLogStream()
    env.run(steps)
    getLogStream()
    _clipsLock.release()


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


def callbackCLIPSPrintFacts(data):
    print ('\nList of Facts:')
    _clipsLock.acquire()
    
    for fact in env.facts():
        print("<Fact-",fact.index,"> ", fact)
    
    _clipsLock.release()


def callbackCLIPSPrintRules(data):
    print ('\nList of Rules:')
    _clipsLock.acquire()
    
    for rule in env.rules():
        print(rule)
    
    _clipsLock.release()


def callbackCLIPSPrintTemplates(data):
    print ('\nList of Templates:')
    _clipsLock.acquire()
    
    for template in env.templates():
        print(template)
    
    _clipsLock.release()


def callbackCLIPSPrintInstances(data):
    print ('\nList of Instances:')
    _clipsLock.acquire()
    
    for instance in env.instances():
        print(instance)
    
    _clipsLock.release()


def callbackCLIPSPrintAgenda(data):
    print ('\nList of Activations:')
    _clipsLock.acquire()
    
    for activation in env.activations():
        print(activation)
    
    _clipsLock.release()


def callbackCLIPSSend(data):
    print ('\nSending Command:')
    _clipsLock.acquire()
    
    env.eval(data.data)
    
    _clipsLock.release()


def callbackCLIPSSendAndRun(data):
    print ('\nSending and Running command:')
    _clipsLock.acquire()
    
    env.eval(data.data)

    resetLogStream()
    env.run(run_steps)
    getLogStream()
    
    _clipsLock.release()


def callbackCLIPSLoadFile(data):
    print ('\nLoading File...')
    filepath = data.data
    
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


def sendCommand(cmdName, params, timeout = defaultTimeout, attempts = defaultAttempts):
    global pubUnknown
    print ('Function name ' + cmdName)
    cmd = Command(cmdName, params)
    func = fmap.get(cmdName)
    if func != None:
        func(cmd)
    else:
        request = PlanningCmdClips(cmd.name, cmd.params, cmd._id, False)
        pubUnknown.publish(request)
    return cmd._id


def setCmdTimer(t, cmd, cmdId):
    t = threading.Thread(target=cmdTimerThread, args = (t, cmd, cmdId))
    t.daemon = True
    t.start()
    return True


def setTimer(t, sym):
    t = threading.Thread(target=timerThread, args = (t, sym))
    t.daemon = True
    t.start()
    return True


def cmdTimerThread(t, cmd, cmdId):
    time.sleep(t/1000)
    _f = env.assert_string('(BB_timer "{0}" {1})'.format(cmd, cmdId))
    print(_f)


def timerThread(t, sym):
    time.sleep(t/1000)
    _f = env.assert_string('(BB_timer {0})'.format(sym))
    print(_f)


def clipsInitialize():
    print ('\nInitializing CLIPS...')
    env.define_function(sendCommand)
    env.define_function(setCmdTimer)
    env.define_function(setTimer)
    
    def_globals = """
    (defglobal
       ?*defaultTimeout* = defaultTimeout
       ?*defaultAttempts* = defaultAttempts)
    """
    env.build(def_globals)
    
    rospack = rospkg.RosPack()

    env.reset()
    resetLogStream()

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
    rospy.Subscriber("/clips_ros/clipspy_assert_command", PlanningCmdClips, callbackCLIPSAssertCommand)
    rospy.Subscriber("/clips_ros/clipspy_eval_command", PlanningCmdSend, callbackCLIPSEvalCommand)
    
    rospy.Subscriber("/clips_ros/clipspy_assert_string", std_msgs.msg.String, callbackCLIPSAssertString)
    rospy.Subscriber("/clips_ros/clipspy_eval_string", std_msgs.msg.String, callbackCLIPSEvalString)
    
    rospy.Subscriber("/clips_ros/clipspy_run",std_msgs.msg.String, callbackCLIPSRun)
    rospy.Subscriber("/clips_ros/clipspy_reset",std_msgs.msg.Bool, callbackCLIPSReset)
    rospy.Subscriber("/clips_ros/clipspy_clear",std_msgs.msg.Bool, callbackCLIPSClear)
    rospy.Subscriber("/clips_ros/clipspy_print_facts",std_msgs.msg.Bool, callbackCLIPSPrintFacts)
    rospy.Subscriber("/clips_ros/clipspy_print_rules",std_msgs.msg.Bool, callbackCLIPSPrintRules)
    rospy.Subscriber("/clips_ros/clipspy_print_templates",std_msgs.msg.Bool, callbackCLIPSPrintTemplates)
    rospy.Subscriber("/clips_ros/clipspy_print_instances",std_msgs.msg.Bool, callbackCLIPSPrintInstances)
    rospy.Subscriber("/clips_ros/clipspy_print_agenda",std_msgs.msg.Bool, callbackCLIPSPrintAgenda)
    
    rospy.Subscriber("/clips_ros/clipspy_send_command",std_msgs.msg.String, callbackCLIPSSend)
    rospy.Subscriber("/clips_ros/clipspy_send_and_run_command", std_msgs.msg.String, callbackCLIPSSendAndRun)
    rospy.Subscriber("/clips_ros/clipspy_load_file",std_msgs.msg.String, callbackCLIPSLoadFile)
    
    #Initialize CLIPS
    clipsInitialize()
    
    rospy.loginfo("CLIPS-ROS Initialized")
    
    #Infinite loop
    rospy.spin()


if __name__ == "__main__":
    main()
