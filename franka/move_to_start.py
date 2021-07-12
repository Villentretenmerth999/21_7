#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray

if __name__ == '__main__':
    rospy.init_node('move_to_start')
    #rospy.wait_for_message('move_group/status', GoalStatusArray)
    commander = MoveGroupCommander('panda_arm')
    #commander.set_named_target('ready')
    #commander.go()
    joint_goal[]=[0,0,0,-1.5,0,2,0]
    #joint_goal[0] = 0
    #joint_goal[1] = 0
    #joint_goal[2] = 0
    #joint_goal[3] = -1.5
    #joint_goal[4] = 0
    #joint_goal[5] = 2
    #joint_goal[6] = 0
    commander.go(joint_goal, wait=True)
