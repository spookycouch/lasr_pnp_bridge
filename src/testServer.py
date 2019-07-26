#!/usr/bin/python

from __future__ import print_function

import roslib
import rospy

import actionlib
import lasr_pnp_bridge.msg
from std_msgs.msg import String

class Random_server(object):
    _feedback = lasr_pnp_bridge.msg.BridgeFeedback()
    _result = lasr_pnp_bridge.msg.BridgeResult()

    def __init__(self, name):
        rospy.loginfo('ExternalServer action server initialised!')
        self._as = actionlib.SimpleActionServer(name, lasr_pnp_bridge.msg.BridgeAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()
        self._plan_manager = rospy.Publisher('planToExec', String, queue_size=10)
    
    def execute_cb(self, goal):
        rospy.loginfo("----------ExternalServer start----------")

        # reset result 
        self._result = lasr_pnp_bridge.msg.BridgeResult()

        # log action and parameters
        rospy.loginfo("action is: " + str(goal.action))
        rospy.loginfo("params are: " + str(goal.params))

        # call the action
        getattr(self, goal.action)(*goal.params)

        # end callback
        rospy.loginfo("----------ExternalServer end----------")
        self._as.set_succeeded(self._result)


    # if the table count is less than 3:
    # print the table number and increment the PNP variable X
    #
    # if not: set the PNP condition "countedAll" to true
    # 
    def gotoAndCount(self, table, tableNo):
        rospy.loginfo("going to " + table + " " + tableNo)
        if int(tableNo) < 3:
            self._result.set_variables = [lasr_pnp_bridge.msg.VariableValue(variable='X', value=str(int(tableNo) + 1))]
        else:
            self._result.condition_event = ["countedAll"]


    # set PNP condition "setTable_1" to initialise variable
    def initTables(self):
        self._result.condition_event = ['setTable_1']
    
if __name__ == '__main__':
    rospy.init_node('ExternalServer')
    server = Random_server(rospy.get_name())
    rospy.spin()
