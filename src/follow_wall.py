#! /usr/bin/env python

# importing all the required libraries and messages
import rospy
import numpy as np 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_basis_project.srv import FindWall, FindWallRequest
from ros_basis_project.msg import OdomRecordAction, OdomRecordGoal, OdomRecordFeedback, OdomRecordResult
import actionlib

class follow_wall(object):

    def __init__(self):
        # define the twist value
        self.move_robot = Twist()

        
        #### find wall service ####

        # create the find wall service
        rospy.wait_for_service('/find_wall_service') 
        self.Find_wall_Client = rospy.ServiceProxy('/find_wall_service', FindWall) 
        self.request = FindWallRequest() 

        rospy.loginfo("Start finding the wall")
        # call the find wall service
        self.result = self.Find_wall_Client(self.request)  

        rospy.loginfo(self.result) 

        rospy.loginfo("End finding the wall")

        #### record_odom action ####

        rospy.loginfo("Start recording the odom")
        # create the connection to the record_odom action server
        self.Record_odom_Client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)

        # waits until the action server is up and running
        self.Record_odom_Client.wait_for_server()
        self.goal = OdomRecordGoal()
        self.goal = ''
        self.Record_odom_Client.send_goal(self.goal, feedback_cb = self.Record_odom_callback)

        #### follow wall ####

        rospy.loginfo("Start following the wall")
        # start the publisher and subscriber 
        self.Follow_wall_Pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.Laser_Sub = rospy.Subscriber('/scan', LaserScan, self.follow_wall_callback)

    def Record_odom_callback(self, msg):
        rospy.loginfo("Current move distance is" + str(msg.current_total) + "meters")

    # define a callback function follow_wall
    def follow_wall_callback(self, msg):
        # define the range size
        size = len(msg.ranges) // 12 

        # define the right and front distance as the minimum var within the range 
        right_dist = min(msg.ranges[180 - size:180 + size])
        front_dist = min(msg.ranges[360 - size: 360 + size])
        n_index = np.argmin(msg.ranges)
        y_index = 180
        # define the linear and angular velocity
        linear = 0.1
        angular = 0.1
        state_result = self.Record_odom_Client.get_state()

        if state_result == 1:
            if (front_dist < 0.5 and front_dist > 0.3):
            # if the cross wall is close, slow down and turn left quickly
                self.move_robot.linear.x = 0.6*linear
                self.move_robot.angular.z = 2*angular
                rospy.loginfo(" The front dist is " + str(front_dist) + ", turn left!")

            # if the cross wall is closer, slow down more and turn left quickly
            elif (front_dist <= 0.3):
                self.move_robot.linear.x = 0.3*linear
                self.move_robot.angular.z = 2*angular
                rospy.loginfo(" The front dist is " + str(front_dist) + ", turn left!")

            elif (right_dist <= 0.2):
                # if the right wall is close, slow down and turn left
                self.move_robot.linear.x = 0.6*linear
                self.move_robot.angular.z = 0.6*angular
                rospy.loginfo(" The right dist is " + str(right_dist) + ", too close, turn left!")

            elif (right_dist > 0.3):
                # if the right wall is far away, slow down and turn right
                self.move_robot.linear.x =  0.6*linear
                self.move_robot.angular.z = -0.6*angular
                rospy.loginfo(" The right dist is " + str(right_dist) + ", too far, turn right!")
            else:
                # adjust and move straight_forward
                self.move_robot.linear.x = linear
                self.move_robot.angular.z = 0
                if (y_index > n_index):
                    self.move_robot.angular.z = -0.6*angular
                    rospy.loginfo("Adjust and Move straight forward!")
                elif (y_index < n_index):
                    self.move_robot.angular.z = 0.6*angular
                    rospy.loginfo("Adjust and Move straight forward!")
                    
            #print(self.move_robot)
        elif state_result >= 2:
                self.move_robot.linear.x = 0
                self.move_robot.angular.z = 0
                rospy.loginfo("Finished the task!")

        self.Follow_wall_Pub.publish(self.move_robot)

if __name__ == '__main__':
    rospy.init_node('follow_wall')
    follow_wall()
    rospy.spin()
