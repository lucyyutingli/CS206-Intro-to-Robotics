
# Q-Learning Project

## Implementation Plan
Team members: Karhan Kayan, Lucy Li

### Q-Learning Algorithm
**Executing the Q-Learning Algorithm**:
We will take random states and actions, and use those to create random trajectories and run those trajectories until the Q-matrix has converged. For every correctly placed dumbell, we will reward 1 point and for every incorrectly placed dumbell, we will reward -1 point. To test this, we will take multiple random trajectories and check these against calculations done by hand to ensure the algorithm is correct. 

**Determing when the Q-Matrix has Converged**:
We will take the q-matrix from the previous trajectory and the q-matrix from the current trajectory and calculate the difference for each cell and sum these differences. We will then check that sum of differences is less than an $\epsilon$ (epsilon). If it is, we will say that the q-learning has converged, otherwise we will continue running trajectories. We will also test this segment by hand by calculating the differencs and sums by hand, then comparing it to what we get through our algorithm. 

**Determine which actions the robot should take to maximize expected reward**:
We will have the robot take the action that has the max q-value for each state. To test this we will run the algorithm and check each action for every state to ensure that the algorithm is indeed taking the max for each state. 

### Robot Perception
**Determining the identities and locations of the three colored dumbbells**:
We will use [this algorithm](https://www.mathworks.com/help/supportpkg/turtlebotrobot/ug/track-and-follow-an-object-with-turtlebot.html) to determine the location of the dumbbells based off their color and to approach them. We may use some additional help from the LiDAR to determine positioning of objects if necesary. To test, we will simply run the algorithm and watch the robot to see if it approaches the correct dumbbell in Gazebo. We will also use print statements for the robot's location and the desired dumbbell's location to make sure that they are correct. 

**Determining the identities and locations of the three numbered blocks**:
We will use `keras_ocr`, as recommended by the project specification. We will use the camera for digit recognition. We will calculate the velocity of the turtlebot turning by using the the center of the camera and the location of the digit within the camera's vision, similar to the method used in the line follower exercise from class 03, in order to center the digit within the robot's camera. We will test it by running the algorithm and seeing if the robot approaches the correct block in Gazebo. We will also use print statements for the robot's location and the desired block's location to make sure that they are correct. 

### Robot Manipulation and Movement
**Picking up and putting down dumbbells with the OpenMANIPULATOR arm**:
We will have the arm centered on the camera's center. Once the robot approaches the correct dumbell, we will lower the arm to pick up the dumbbell. This will all be done using the MoveIt ROS package as in class 08. To test, we will run the program and watch the turtlebot in Gazebo to ensure it picks up the dumbbell correctly.

**Navigating to the appropiate locations to pick up and put down the dumbbells**:
After we pick up the dumbbell, we will keep it at the camera's center (above the camera, out of its vision). We will then locate the block the turtlebot wants to go to. We will have the turtlebot turn towards the wanted block and move forward. This can all be done using `\odometry` and `\cmd_vel`. To test this, we will run the program and watch the robot in Gazebo in order to ensure that it is going to the correct location. We will also use print statements for the robot's location, the dumbbells location, and the desired block's location. 

### Timeline 
We will have the Q-learning algorithm done by May 5, and Robot Perception and Robot manipulation and movement done by the due dayte, May 12.

---

# Writeup

## GIF
![](q_learning_gif.gif)

## Objectives Description
The goal of this project is to use Q-Learning to train a robot to put the colored dumbbells in front of the correct blocks, in order to maximize the reward. To do this, we told the robot what to do at each state using using reinforcement learning.

## High-Level Description
We were given the rewards for each state of the robot. Using these rewards, we trained the q-matrix using the Q Learning Algorithm. After the training phase, we used the trained q-matrix to determine the best action for each state that the robot is in. Based on the best action the robot uses computer vision and PID to move towards the target dumbell. It then uses kinematics to pick up the dumbell. Then we used keras_ocr to process the camera feed of the robot to determine where the corresponding block is to our dumbell. The robot then moves towards the block to let go of the dumbell. Both when moving towards the dumbells and the blocks, we check the minimum distance to objects using lidar. Once the minimum was reached, we put the dumbbell down or lift the dumbell up depending on which stage of the process we are in. This is done with the OpenManipulator GUI to control the robots arm. We then repeat this for each dumbbell as necessary.

## Q-Learning Algorithm Description
- **Selecting and executing actions for the robot to take:**
In the function `initialize_possible_actions()` we set up the matrix of possible actions from each state and the matrix of what action follows given a state action pair. These are called `self.possible_actions` and `self.state_action_matrix` respectively. In the `Train_Q()`, we state with state 0 and take a random action at each step. Based on the aforementioned matrices, we determine what the next state should be. 

- **Updating the Q-matrix:**
In the function `Train_Q()`, we update the (state,action) pair in the q-matrix using the q updates provided in class. We receive the reward that is used in the update from the `/q_learning/reward` topic. 


- **Determing when to stop iterating through the Q-learning algorithm:**
This is determined in the `check_convergence()` function. We set `epsilon = 1e-1` and determine if the rolling average of the last 500 updates is smaller than this epsilon. If so, we declare that we have converged. We set a hard limit 10000 on the maximum number of iterations.

- **Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot:**
We save the q matrix as a .npy file after running training. We then load this file in `detector.py` to choose the best actions. At any given state, we take the action that maximizes the Q value of that state action pair. This is done in `choose_next_action()` where we choose the column that maximizes the row corresponding to a state. 

## Robot Perception Description
- **Identifying the locations and identities of each of the colored dumbbells:**
We first wrote the `get_mask()` function in detector.py to set the limits for each color. Color detection works in the same way as it did in the line follower code. So, we just get the appropriate cv2 mask for the color of dumbell we are trying to find. This identification is done in the `take_dumbbell` function. 

- **Identifying the locations and identities of each of the numbered blocks:**
Block finding is done by digit prediction using keras_ocr in the function `find_block()`. Each time we look for a block we try to see if we recognize the digit we are trying to find. The Keras pipeline gives the location of the recognized digit, which we try to center in the camera. Also since it can misinterpret some digits (1 as l for instance), we also have a dictionary of things it can misinterpret as called `digit_interpretations`.

## Robot Manipulation and Movement
- **Movement to right spot:**
    This section of the code is at `take_dumbell()` function of the `detector.py` file. We calculate the error between the camera sensor and the center of the colors we are trying to find. Then using pid we move those closer together. We also use the LiDAR to detect the distance from the dumbbells, so the robot knows when to stop moving forward when it's at the proper distance. A technicality here is that we use the maximum of the last couple of scans since the robot might be affected by the noise. 

- **Picking up the dumbbell:**
    This section is at the top of the `detector.py` file. We used the OpenManipulator GUI from Class Meeting 8 to test which angles would work best with the dumbbells. Then, we used the MoveIt package to instruct the robot to go to certain angles for the specific situation. For example, there is a different position for prepping to picking up a dumbbell, actually picking it up, and putting it back down. The kinematics of this process is in the `pick_up()` function in detector.py.

- **Moving to the desired destination with the dumbbell:**
    This section of the code is at `find_block()` function of the `detector.py` file. While we cannot see the blocks, we continue to rotate. Once the correct block is perceived, we recognize it, then center it in the camera using PID error between the camera center and the digit location. We then move towards the block and stop the correct distance, which we determined by trial and error, before putting the dumbbell down. 

- **Putting the dumbbell back down at desired destination:**
    This section of the code is at the `let_go()` funtion of the `detector.py` file. Once the robot is at the desired destination, move the robot to the correct angles to put it back down. We actually use the same angles we do in the **picking up the dumbbell** section, however we make sure that the arm of the robot is in the right angle before changing the gripper angle, which will release the dumbbell. 

## Challenges
Some challenges we encountered were issues with our camera. While we were working on the assignment, our camera feed suddenly stopped working and we fixed it by getting some extra help and debugging. Another challenge was figuring out how to divide each node and also how to combine them. We solved this by creating one file for training and another for executing. This allowed us to test each phase individually to ensure each section worked properly. We also had issues with the sleeps, and having to understand how they interacted with the camera. Eventually we had to reformat our entire code in order to ensure that the camera was updating properly. Finally, we had to figure out the right velocities and kp's in order for the robot to move at the right speed to prevent it from picking up or dropping dumbbells too soon. This just mostly took trial and error, until we eventually tweaked it to a good position. 

## Future work
If we had the extra time, I would want to maybe make the robot move a bit quicker, or have a more accurate position when putting down or picking up the blocks. Since it can kind of vary based off where the robot is currently, the dumbbells weren't always dropped off in the same spot. Also the digit recognition part is a significant bottleneck of the system. Finding a faster method for that would speed up the process by a large margin. Another area of improvement is the robots sensitivity to lidar noise. This sensitivity can be adjusted by adjusting the scanner parameters. 

## Takeaways
- Separating files allows for easier testing and debugging, and also allows for clarity on each persons' task. Having different sections of the project in different files made it really easy to pinpoit problems and solve bugs much quicker. This also allowed each person to work on their own area of expertise. 
- The GUI is super helpful for figuring out angles. I was really worried about finding the correct angles, but using trial and error with the GUI made it really easy to figure out the proper angles to pick up the dumbbells. It went by much faster than I thought.
- The Q-Matrix can be applied to anything with a reward system. Although we only focused on the dumbbells, we can see how the q-matrix could be used with any game/system. This can allow for future development whether it be a small school project, or more practical systems for use in hospitals or factories.
