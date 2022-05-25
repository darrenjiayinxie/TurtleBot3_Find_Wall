#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np 
from ros_basis_project.srv import FindWall, FindWallResponse


class find_wall(object):

    def __init__(self):
        # define the twist value
        self.move_robot = Twist()
        self.Pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.Srv = rospy.Service('/find_wall_service', FindWall, self.find_wall_callback) 
        

    def hampel(self, x, k, t0 = 3):
        '''adapted from hampel function in R package pracma
        x= 1-d numpy array of numbers to be filtered
        k= number of items in window/2 (# forward and backward wanted to capture in median filter)
        t0= number of standard deviations to use; 3 is default
        '''
        n = len(x)
        
        y = x #y is the corrected series
        L = 1.4826
        mask = np.full((n, 1), True)
        for i in range((k + 1),(n - k)):
            if np.isnan(x[(i - k):(i + k+1)]).all():
                continue
            x0 = np.nanmedian(x[(i - k):(i + k+1)])
            S0 = L * np.nanmedian(np.abs(x[(i - k):(i + k+1)] - x0))
            if (np.abs(x[i] - x0) > t0 * S0):
                y[i] = np.inf
                mask[i] = False
        return y, mask

    def find_wall_callback(self, request):
       

        # define the linear and angular velocity
        linear = 0.1
        angular = 0.1
        
        # rotate the robot 
        while True:
            # identify the shortest laser ray
            msg = rospy.wait_for_message('/scan', LaserScan)
            size = len(msg.ranges) // 24
            y = np.array(msg.ranges).reshape(-1,1)
            x = np.arange(0, 720, 1).reshape(-1, 1)
            filtered, mask = self.hampel(y, 72, 1)
            
            #f = np.poly1d(np.polyfit(x[mask], filtered[mask], 4))
            #y_ = f(x)
            #print(filtered)
            n_ray_index = np.argmin(filtered[180:540]) + 180
            #n_ray_index = np.argmin(msg.ranges[180:540]) + 180
            

            # how to avoid the non-wall obstacles?
            # idea: fit a curve of the distance to the wall and omit the data from the obstacles

            
            if (n_ray_index > 360 + size):
               
                self.move_robot.linear.x = 0.3*linear
                self.move_robot.angular.z = 1*angular
                rospy.loginfo("Find the closest wall: index - " + str(n_ray_index) + " turn right")
            elif (n_ray_index < 360 - size):
              
                self.move_robot.linear.x = 0.3*linear
                self.move_robot.angular.z = -1*angular
                rospy.loginfo("Find the closest wall: index - " + str(n_ray_index) + " turn left")
            else:
                break

            #rospy.loginfo(self.move_robot)
            self.Pub.publish(self.move_robot)

        rospy.loginfo("Facing the nearest wall")
        # move forward to the closest wall
        while True:
            msg = rospy.wait_for_message('/scan', LaserScan)
            n_ray_dist = min(msg.ranges[350:370])
            
            self.move_robot.angular.z = 0
            if (n_ray_dist <= 0.25):
                self.move_robot.linear.x = - 0.3*linear
                rospy.loginfo("Move further from the wall: distance - " + str(n_ray_dist))
            elif (n_ray_dist > 0.3):
                self.move_robot.linear.x = 0.3*linear
                rospy.loginfo("Move close to the wall: distance - " + str(n_ray_dist))
            elif (n_ray_dist <= 0.3 and n_ray_dist > 0.25):
                break
            #rospy.loginfo(self.move_robot)
            self.Pub.publish(self.move_robot)

        # align right side to wall
        while True:
            msg = rospy.wait_for_message('/scan', LaserScan)
            n_ray_index = msg.ranges.index(min(msg.ranges))
            rospy.loginfo("Adjust pose: index - " + str(n_ray_index))

            if (n_ray_index < 270 - size):
                self.move_robot.linear.x = 0*linear
                self.move_robot.angular.z = -angular
            elif (n_ray_index > 270 + size):
                self.move_robot.linear.x = 0.01*linear
                self.move_robot.angular.z = angular
            else:
                break

            #rospy.loginfo(self.move_robot)
            self.Pub.publish(self.move_robot)

        response = FindWallResponse()
        response.wallfound = True
        return response

if __name__ == '__main__':

    rospy.init_node('find_wall')
    find_wall()
    rospy.spin() # mantain the service open.