# Particle Filter Localization Project

## Implementation Plan
Team Members: Dante Gil-Marin, Lucy Li

### `initialize_particle_cloud`
We will use `random_sample()` in order to draw a random sample for one particle, which we will then add to the `particle_cloud` array. This sample will come from distribution of all locations in the room with equal weights. This will be done in a for loop until we have reached the desired amount of particles. To test, we will print out the information stored in the `particle_cloud` array to ensure that it was initialized properly.

### `update_particles_with_motion_model`
We will take the movement of the robot in terms of change in x, y, and angle, then apply these changes to each particle in the array. This will be done in a for loop. To test this, we will print the `particle_cloud` array again.

### `update_particle_weight_with_measurement_model`
We will first create a new array named `weighted_particles` that consists of the expected sensor readings for each particle in `particle_cloud`, given the map. Then, we will get the true sensor readings from the robot, and apply the weight formula to each element of the new array. We will test this by applying this to a small particle cloud, then comparing the printed information to calculations we will do by hand.

### `normalize_particles` and `resample_particles`
To normalize the the particles' weights, we will take the sum of the weights in `weighted_particles`, then divide each particle's weight in `weighted_particles` by this sum. To resample the particles, we will do `random_sample()` again on `particle_cloud`, however this time the distribution will be based off their weights in `weighted_particles`. We will test this by making a small particle cloud and running this algorithm on it. We will compare the results to calculations we do by hand.

### `update_estimated_robot_pose`
We will estimate the robot pose by taking the weighted average. We will use the weights from `weighted_particles` to create the weighted average of `particle_cloud`.

### noise
When we update the particles in `update_particles_in_motion_model`, instead of updating the position of the particles with the change in the x, y, angle from the robot, we will take a random sample from a Gaussian around our change and use that to update the position.

### timeline
We want everything up to `update_particle_weight_with_measurement_model` to be done by Monday, April 19. Everything following should be finished by Sat, April 24 to give us time to polish anything that needs to be looked at. All the functions will be done in order with pair programming.

---

# Writeup

# GIF
![Gif](particle_filter.gif)

## Objectives Description
The goal of the project is to find the true postion of the robot within a given map of the house it is in. We accomplish this through Monte Carlo localization, implemented through particle filtering. We use many particles and their own positions and angles, checking their surrounding obstacles to compare to the robot's own surrounding obstacles. 

## High-Level Description
At a high level, the particle filtering algorithm we implemented has 7 components: initialization of the particle cloud, movement model, measurement model,resampling, adding noise, updating estimated pose, optimizing paramenters. To initialize the particle cloud, we randomly select locations on the map, ensuring they are within the house and not on an obstacle. These locations are paired with random orientations to give a complete pose. Then in the movement model, we update all our particles to reflect the change in position and orientation of the robot. Next in the measurement model, we use the laser range finder to detect the distance to and direction of objects surrounding the actual robot. We compare this data to equivalent measurements given by the particle locations on the map. The similarity measure resulting from this is used to weight the particles. The weights are first normalized and then used to resample the particles, using random weighted sampling. [incorporate noise]. [update robot pose]. [optimize].

## Main Steps
1. **Initialization of Particle Cloud**
    We initialize the particle cloud within the ParticleFilter class. The function we use, `initialize_particle_cloud` is defined underneath the function call. In `initialize_particle_cloud`, within a while loop, we define two variables, x and y, that randomly select an x and y from the map respectively. We check to make sure that the x and y are valid locations, meaning they are within the house and not on an obstacle, by checking the OccupancyGrid data. If the position has a value of 100 on the OccupancyGrid data, we create a new Particle using this location along with a random orientation which is taken using `randrange()` then append this particle to the `particle_cloud` array. This completes the step of initializing the particle cloud.

2. **Movement Model**
    We define the `update_particles_with_motion_mode` function at the end of the file, within the ParticleFilter class. In the movement model, we take the difference between the `self.odom_pose` and `self.odom_pose_last_motion_update`. Using this, we created three variables, x_change, y_change, and ang_change to reflect the x, y, and angle change in the robot's position. We use a for loop to add these changes to each of the particles in the particle cloud.

3. **Measurement Model**
    The measurement model is split up using multiple files. We use the `likelihood_field.py` given in class for generating the likelihood field. We define `update_particle_weights_with_measurement_model` towards the end of the file within the ParticleFilter class. We use the Likelihood Fields for Range Finders algorithm given in class. For each particle in the `particle_cloud`, we set the weight to 1, then adjust the weight accordingly, checking 8 directions within 360 degrees. We use the Likelihood Field for Range Finders algorithm, so we determine the x and y separately, and use that to calculate the closest distance of an object useing `LikelihoodField.get_closest_obstacle_distance()`, and finally recompute the weight, which is `q`. This recomputes the weights. 

4. **Resampling**
    To resample, we used `draw_random_sample()` which we defined ourself at the top of the particle_field.py file. We use `np.random.choice` which intakes an array, the size of the sample, and the weights for the distribution. We simply call this function on the `particle_cloud` array and the size of the array in order to do our resampling. This completes the resampling of particles with new weights. 
    
5. **Incorporation of Noise**
    Based off recommendations given to us by teaching staff, we inserted noise wihtin the `update_particles_with_motion_model()` function. We used `np.random.normal(0,1)` in order to pick a number between -1 and 1 on a normal distribution to add to our x, y, and orientation. This helps offset the noise from Gazebo itself. 

6. **Updating estimated robot pose**
    Based off recommendations from office hours, in the function `update_estimated_robot_pose()` we take the average of our particles' x, y, and angle fields from the `particle_cloud` and set those to be our updated esimated pose. This requires adding the x's, y's, and angles to each other respectively, before dividing all by the number of particles. This helps reach our final goal of estimating the robot's pose.

7. **Optimize parameters**
    We wanted to ensure that we had ample particles to estimate the robot's position, however we didn't want to run into the issue where it took too long to compute. In order to compromise these things, we decided to use 8 directions instead of checking all 360 angles. We thought this would be good coverage in all directions, in addition to the amount of particles. We started lower, but found that the particles became more accurate as we added more, so we ended with 5000 particles. For all standard deviation, we started larger, before slowly decreasing it for all standard deviations we used. As we decreased, we found that our particles become more accurate, so we decided to end with 0.1 which was small but still had the right affect. 

## Challenges
The first challenge was getting familiar with the base code. Although we had previous exposure in class, there were additional factors that we had to understand in order to get started. The Likelihood Field algorithm also was a little difficult for us to understand, however it took us a while understand how to implement it more than anything. We also struggled with mapping the particles to the map properly because `OccupancyGrid()` had a different position in comparison to the map. One of the smaller challenges was figuring out the best way to deal with `nan` when it was encountered in `get_closest_obstacle()`, but we eventually discovered we were simply comparing it incorrectly. Finally, figuring out the correct noise was also fairly difficult. We didn't realize our noise was too large, which led us to having difficulty in locating the robot. 

## Future Work
We would want to try to maximize the particles or the angles more. Since the estimate becomes more accurate as the particles increase, that would be something we want to focus on. Additionaly, fine tuning the standard deviations further could help our results become more accurate if needed. We also want to speed up the rate of convergence, however we know that is difficult based off multiple different factors. 

## Takeaways
1. We learned a lot in understanding how ros works and how it interacts with Gazebo and RViz. This will be super helpful in the future for understanding how each part is integrated, especially because we will encounter these programs again. It is best to discuss these with your partner, cause each can give the other insight that they had not original thought of.
2. For smaller projects like this, it is best to go over sections together rather than coding different sections separately. Of course, when there is limited time and a large amount of code this isn't very feasible, however we found it was most helpful when we were looking together. 
3. We have a deeper understanding of Monte Carlo Localization. This will be helpful in future work if we encounter localization with range finders again. Understanding the algorithm also got us more familiar with the mathematical descriptors, which will be helpful looking at other algorithms in the future. 

