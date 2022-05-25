#! /usr/bin/env python

import rospy
from ros_basis_project.msg import OdomRecordAction, OdomRecordFeedback, OdomRecordResult
import actionlib
from nav_msgs.msg import Odometry
import time
import math
from geometry_msgs.msg import Point
class record_odom(object):

    def __init__(self):
        # initialize variables
        self.curr_pose = Point()
        self.prev_pose = Point()
        self._result = OdomRecordResult()
        self._feedback = OdomRecordFeedback()
        self.complete_lap = 6

        # creates the action server
        self._as = actionlib.SimpleActionServer('/record_odom', OdomRecordAction, self.execute_callback, False)
        self._as.start()
        
    
    def update_dist(self):
        curr_move = math.sqrt((self.curr_pose.x - self.prev_pose.x)**2 + (self.curr_pose.y - self.prev_pose.y)**2)
        self._feedback.current_total = self._feedback.current_total + curr_move

    def update_curr_pose(self, msg):
        self.curr_pose.x = msg.pose.pose.position.x
        self.curr_pose.y = msg.pose.pose.position.y
        self.curr_pose.z = math.acos(msg.pose.pose.orientation.w)*2

    def update_prev_pose(self, msg):
        self.prev_pose.x = msg.pose.pose.position.x
        self.prev_pose.y = msg.pose.pose.position.y
        self.prev_pose.z = math.acos(msg.pose.pose.orientation.w)*2

    def execute_callback(self, goal):
        
        success = True
        rospy.loginfo('action server called')

        # initialize the prev_pose and curr_pose
        msg = rospy.wait_for_message('/odom', Odometry)
        self.update_curr_pose(msg)
        self.update_prev_pose(msg)
        
        while True:
            # receive Odometry 
            msg = rospy.wait_for_message('/odom', Odometry)

            # update curr pose
            self.update_curr_pose(msg)

            # update result
            self._result.list_of_odoms.append(self.curr_pose)

            # update feedback
            self.update_dist()
            self._as.publish_feedback(self._feedback)

            #
            if (self._feedback.current_total > self.complete_lap):
                rospy.loginfo('Complete one lap')
                break
            # update prev pose
            self.update_prev_pose(msg)

            time.sleep(1)

        if success:
            self._as.set_succeeded(self._result)
        

if __name__ == '__main__':
    rospy.init_node('record_odom')
    record_odom() 
    rospy.spin()