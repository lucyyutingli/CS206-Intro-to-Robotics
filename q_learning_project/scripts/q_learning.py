#!/usr/bin/env python3

import rospy
import numpy as np
from q_learning_project.msg import QLearningReward, QMatrix, RobotMoveDBToBlock 
import os

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))


        #Adding our things
        self.rolling_average = 0
        self.Q = np.zeros((64,9))
        self.prev_Q = np.zeros((64,9))
        self.initialize_possible_actions()
        self.rolling_average = 0
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.process_reward)
        self.matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)

    #saves the q matrix to an npy file
    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        print('saving')
        with open('qmatrix.npy', 'wb') as f:
            np.save(f, self.Q)
        return 

    #get lists of possible actions from each state and the state+action -> state matrix
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


    #reward callback function
    def process_reward(self, data):
        self.reward = data.reward
        self.reward_received = 1

    # check if Q learning has converged
    def check_convergence(self):
        epsilon = 1e-1
        if self.rolling_average < epsilon:
            return True
        else:
            return False

    # Train the Q matrix 
    def Train_Q(self):
        alpha = 1
        gamma = 0.5
        state = 0

        #iterate 
        for i in range(10000):
            #check for convergence
            if i > 1000 and self.check_convergence():
                print('converged')
                break
            #if no actions to take reset. 
            if len(self.possible_actions[state]) == 0:
                state = 0
            
            #choose a random action and find the next state
            action = np.random.choice(self.possible_actions[state])
            next_state = self.state_action_matrix[state][action]
            move_msg = RobotMoveDBToBlock()
            move_msg.robot_db = self.actions[action]['dumbbell']
            move_msg.block_id = self.actions[action]['block']
            r = rospy.Rate(100)
            #publish the action
            while True:
                # make sure that the number of connections is positive so that no message is lost
                if self.action_pub.get_num_connections() > 0:
                    self.action_pub.publish(move_msg)
                    break
                # if no connetions, wait until the connection is established.
                else:
                    r.sleep()
            self.reward_received = -1
            while self.reward_received == -1:
                r.sleep()
            next_value = np.max(self.Q[next_state])

            delta = alpha * (self.reward + gamma * next_value - self.Q[state, action])
            
            #Add the change in the q matrix to the rolling average of changes for convergence test
            self.rolling_average = (1 - 1/500) * self.rolling_average + abs(delta) / 500
            print(delta, self.rolling_average)

            #update the q matrix
            self.Q[state, action] += delta
            state = next_state
        print(self.Q)
            

if __name__ == "__main__":
    node = QLearning()

    node.Train_Q()

    node.save_q_matrix()