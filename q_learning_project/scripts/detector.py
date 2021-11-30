#!/usr/bin/env python3

import rospy,cv_bridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import keras_ocr
from q_learning_project.msg import RobotMoveDBToBlock
from sensor_msgs.msg import LaserScan
import moveit_commander
from collections import deque
import os


path_prefix = os.path.dirname(__file__) + "/action_states/"

class Detector:

        def __init__(self):
            rospy.init_node('detector')

            
            #Action selection these are mostly the same as in q_learning.py
            self.qstate = 0
            self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
            colors = ["red", "green", "blue"]
            self.actions = np.loadtxt(path_prefix + "actions.txt")
            self.actions = list(map(
                lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
                self.actions
            ))
            self.states = np.loadtxt(path_prefix + "states.txt")
            self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))



            self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)
            self.state = 0

            #load the trained q matrix
            with open('qmatrix.npy', 'rb') as f:
                self.Q = np.load(f)

            self.initialize_possible_actions()

            #initialize the keras pipeline for digit recognition
            self.pipeline = keras_ocr.pipeline.Pipeline()
            self.digit_interpretations = {'1': ['1','l'], '2' : ['2', 's', 'ss'], '3' : ['3'] }
            self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
            self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

            self.lastscans = deque([1e9]*3)
            self.state = 'nothing'

            # set up ROS / OpenCV bridge
            self.bridge = cv_bridge.CvBridge()    

            # subscribe to the robot's RGB camera data stream
            rospy.Subscriber('camera/rgb/image_raw',
                    Image, self.image_callback)

            #rospy.Subscriber("/q_learning/robot_action", 
            #        RobotMoveDBToBlock, self.action_callback)

            rospy.Subscriber('/scan', LaserScan, self.scan_callback)

            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.vel_msg = Twist()

            rospy.sleep(1)

        #This is the same as in q_learning.py. Answers what are the possible actions from each state
        # and how can we determine the next state from a state action pair
        def initialize_possible_actions(self):
            self.possible_actions = []
            self.state_action_matrix = []
            for state in range(64):
                self.possible_actions.append([])
                self.state_action_matrix.append([0]*9)
                for next_state in range(64):
                    action = int(self.action_matrix[state, next_state])
                    if action != -1:
                        self.possible_actions[state].append(action)
                        self.state_action_matrix[state][action] = next_state


        #choose the next best action based on Q matrix
        def choose_next_action(self):
            self.action = np.argmax(self.Q[self.qstate])
            self.dumbell = self.actions[self.action]['dumbbell']
            self.block = str(self.actions[self.action]['block'])
            self.num_states += 1
            

        # Prep arm for pickup angles
        def run_prep(self):
            arm_joint_goal = [0.0, 0.55, 0, -0.6]
            gripper_joint_goal = [0.012, 0.012]

            self.move_group_gripper.go(gripper_joint_goal, wait=True)
            self.move_group_arm.go(arm_joint_goal, wait=True)

            self.move_group_arm.stop()

        # Pick up the object
        def pick_up(self):
            arm_joint_goal = [0, -0.196, 0.001, -0.6]
            gripper_joint_goal = [0.009, 0.009]

            self.move_group_gripper.go(gripper_joint_goal, wait=True)
            self.move_group_arm.go(arm_joint_goal, wait=True)

            self.move_group_arm.stop()

        # Put down the object
        def let_go(self):
            arm_joint_goal = [0.0, 0.65, 0, -0.7]
            gripper_joint_goal = [0.012, 0.012]

            self.move_group_arm.go(arm_joint_goal, wait=True)
            self.move_group_gripper.go(gripper_joint_goal, wait=True)

            self.move_group_arm.stop()

        """
        Gets the mask for cv2 to use to determine where the dumbell is.
        The mask is based on the color and hsv values are used for that color.
        """
        def get_mask(self, color, hsv):

            #what is the color of the dumbell
            if color == 'red':
                lower_red = np.array([0,120,70])
                upper_red = np.array([10,255,255])
                mask1 = cv2.inRange(hsv, lower_red, upper_red)

                lower_red = np.array([170,120,70])
                upper_red = np.array([180,255,255])
                mask2 = cv2.inRange(hsv,lower_red,upper_red)

                return mask1 + mask2
            elif color == 'green':
                lower_green = np.array([40,40,40])
                upper_green = np.array([70,255,255])

                return cv2.inRange(hsv, lower_green , upper_green)

            elif color == 'blue':
                lower_blue  = np.array([100, 150, 0])
                upper_blue = np.array([140, 255, 255])

                return cv2.inRange(hsv, lower_blue  , upper_blue)     

        # Move towards a dumbell to pick it up
        # This is where the perception and pid occurs
        def take_dumbbell(self, hsv):
            mask = self.get_mask(self.dumbell, hsv)

            h, w, d = self.image.shape
            M = cv2.moments(mask)

            #pid for angular velocity

            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                #center the dumbell in the camera
                error = (w/2 - cx)/(w/2)
                kp_angular = 0.9
                self.vel_msg.angular.z = kp_angular*error
            
                kp_linear = 0.2
                self.vel_msg.linear.x = kp_linear * self.min_dist 
                self.vel_pub.publish(self.vel_msg)

            #if the dumbell is not seen then turn and look for it 
            else:
                self.vel_msg.angular.z = 0.5
                self.vel_msg.linear.x = -0.2
                self.vel_pub.publish(self.vel_msg)
        
        # Find the block that we are trying to put our dumbell to 
        def find_block(self):
            prediction_groups = self.pipeline.recognize([self.image])
            print('finished prediction')

            found_block = False
            h, w, d = self.image.shape

            
            for tp in prediction_groups[0]:
                print((tp[0]))

                #If the prediction is the digit we are looking for 
                #center it in the camera using pid
                if tp[0] in self.digit_interpretations[self.block]:
                    found_block = True
                    rect = tp[1]
                    cx = (rect[0,0] + rect[1,0]) / 2
                    cy = (rect[1,1] + rect[2,1]) / 2
                    error = (w/2 - cx)/(w/2)
                    kp_angular = 0.7
                    self.vel_msg.angular.z = kp_angular*error
                    self.vel_pub.publish(self.vel_msg)

            #try to move towards the block
            if self.state == 'move to block':
                self.vel_msg.linear.x = 0.37
                self.vel_pub.publish(self.vel_msg)

            #if the block is not found turn and continue looking for it
            elif not found_block:
                self.vel_msg.angular.z = 0.6
                self.vel_msg.linear.x = -0.1
                self.vel_pub.publish(self.vel_msg)

            rospy.sleep(1)
            self.vel_msg.angular.z = 0
            self.vel_msg.linear.x = 0
            self.vel_pub.publish(self.vel_msg)
            rospy.sleep(1)

            return found_block

    

        #set the last couple of scans
        def scan_callback(self, data):
            self.min_dist = min(min(data.ranges[0:90]), min(data.ranges[270:]))
            self.lastscans.pop()
            self.lastscans.appendleft(self.min_dist)

        def scan_processing(self):
            mx = max(self.lastscans)
            print(self.lastscans)
            
            #if we are trying to move towards a dumbell and close to it, then stop and lift
            if 0.17 <= mx <= 0.2 and self.state == 'move dumbell':
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.vel_pub.publish(self.vel_msg)
                self.state = 'lift'

            #if we are close to a block stop and drop
            if mx <= 0.58 and self.state == 'move to block':
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.vel_pub.publish(self.vel_msg)
                self.state = 'drop'

        #set the camera feed
        def image_callback(self, msg):
            self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        #process the image to find the next action we should take
        def image_processing(self):
            hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
            
            if self.state == 'move dumbell':
                self.take_dumbbell(hsv)


            # Use kinematics to pick up the dumbell
            elif self.state == 'lift':
                self.pick_up()
                rospy.sleep(1)
                
                self.vel_msg.linear.x = -0.5
                self.vel_pub.publish(self.vel_msg)
                rospy.sleep(1)
                
                self.vel_msg.linear.x = 0
                self.vel_pub.publish(self.vel_msg)
                self.state = 'find block'
                
            #try to find our block
            elif self.state == 'find block' or self.state == 'move to block':
                if self.find_block():
                    self.state = 'move to block'

            #lower the dumbell and choose the next action from the q matrix
            #this sets a new dumbell and block to find
            elif self.state == 'drop':
                self.let_go()
                rospy.sleep(1)
                self.qstate = self.state_action_matrix[self.qstate][self.action]
                self.vel_msg.linear.x = -1
                self.vel_pub.publish(self.vel_msg)
                rospy.sleep(1)
                self.choose_next_action()
                self.state = 'move dumbell'

            else:
                return 
        

        def run(self):
            r = rospy.Rate(5)

            self.num_states = 0
            self.run_prep()
            self.state = 'move dumbell'
            self.choose_next_action()
            print(self.dumbell, self.block)

            while not rospy.is_shutdown():
                #if we have put three dumbells stop
                if self.num_states >= 4:
                    self.vel_msg.linear.x = 0
                    self.vel_pub.publish(self.vel_msg)
                    break
                print(self.state)
                print(self.dumbell, self.block)
                
                self.image_processing()
                self.scan_processing()

                r.sleep()       

if __name__ == '__main__':
    detector = Detector()

    detector.run()