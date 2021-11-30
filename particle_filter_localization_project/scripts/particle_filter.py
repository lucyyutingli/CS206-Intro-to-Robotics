#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random, randrange, choices, uniform
import likelihood_field as lf
from copy import deepcopy



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(size_of_sample, particle_array):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities. 
    We recommend that you fill in this function using random_sample.
    """
    
    """ We decided to use np.random.choice() which allows us to pick from items with associated proababilities"""

    # Extract the weight array
    weight_array = [p.w for p in particle_array]
    
    # Return the new random sample using the given weights
    new_array = choices(particle_array, k=size_of_sample, weights=weight_array)
    array = [deepcopy(p) for p in new_array]
    return array


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 5000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        



        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # Give the map some time to initialize
        rospy.sleep(2)


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True

        # Add the Likelihood Field
        self.likelihood_field = lf.LikelihoodField(self.map)


    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        
        # TODO
        # Obtain height and width from map
        h = self.map.info.height - 1
        w = self.map.info.width - 1


        while (len(self.particle_cloud) < self.num_particles):
            y = randrange(h)
            x = randrange(w)
 
            # Check if the coordinate is in a valid location, append to particle_cloud if so
            if self.map.data[x + y * (w + 1)] == 0:
                angle = quaternion_from_euler(0,0, uniform(0, 2 * math.pi))
                new_angle = Quaternion(angle[0], angle[1], angle[2], angle[3])

                # Finalize pose and angle, append particle 
                particle_pose = Pose(Point((x - 197) * self.map.info.resolution, (y - 197) * self.map.info.resolution, 0),new_angle)
                new_particle = Particle(particle_pose,1)

                self.particle_cloud.append(new_particle)


        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        
        # TODO
        # Add all particle weights together, then divide each weight by the sum
        sum = 0


        for i,p in enumerate(self.particle_cloud):
            sum += self.particle_cloud[i].w


        weight_sum = 0
        for i,p in enumerate(self.particle_cloud):
            self.particle_cloud[i].w = float(self.particle_cloud[i].w/sum)
            weight_sum += p.w




    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # TODO
        # Draw a new weighted random sample from the particles
        self.particle_cloud = draw_random_sample(len(self.particle_cloud), self.particle_cloud)
        return      




    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # TODO
        # Take the average of all position and orientations of the particles in the particle cloud
        x_total = 0
        y_total = 0
        angle_total = 0

        # Get the averages of x, y, and angles of all particles
        for i,p in enumerate(self.particle_cloud):
            x_total += p.pose.position.x
            y_total += p.pose.position.y
            angle_total += get_yaw_from_pose(p.pose)

        x_final = x_total / len(self.particle_cloud)
        y_final = y_total / len(self.particle_cloud)
        angle_final = angle_total / len(self.particle_cloud)


        # Update the robot's position and orientation
        self.robot_estimate.position.x = x_final
        self.robot_estimate.position.y = y_final
        new_angle = quaternion_from_euler(0,0,angle_final)

        self.robot_estimate.orientation = Quaternion(new_angle[0], new_angle[1], new_angle[2], new_angle[3])
    
    def update_particle_weights_with_measurement_model(self, data):

        
        # TODO
        # 8 directions to check direction for
        directions = [0, 45, 90, 135, 180, 225, 270, 315]

        # Compute Likelihood Fields for Range Finders algorithm
        for i,j in enumerate(self.particle_cloud):
            q = 1
            for k in directions:
                if data.ranges[k] < 3.5:
                    x = j.pose.position.x + data.ranges[k]*(math.cos(get_yaw_from_pose(j.pose) + math.radians(k)))
                    y = j.pose.position.y + data.ranges[k]*(math.sin(get_yaw_from_pose(j.pose) + math.radians(k)))
                    dist = self.likelihood_field.get_closest_obstacle_distance(x,y)
                    if math.isnan(dist):
                        dist = 0.0000000000000000000000000000000000000000000000000000000000000000001
                        q = q * dist
                    else:
                        q = q * (lf.compute_prob_zero_centered_gaussian(dist, 0.5))
            self.particle_cloud[i].w = q


    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO
        # Get change for x, y, and angle, then apply to all particles in the particle cloud
        x_change = self.odom_pose.pose.position.x - self.odom_pose_last_motion_update.pose.position.x
        y_change = self.odom_pose.pose.position.y - self.odom_pose_last_motion_update.pose.position.y
        ang_change = get_yaw_from_pose(self.odom_pose.pose) - get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        v = math.sqrt(x_change**2 + y_change**2)


        # Set new position with some added noise
        for i,j in enumerate(self.particle_cloud):
            new_angle = quaternion_from_euler(0,0,get_yaw_from_pose(j.pose) + ang_change + np.random.normal(0,.1))
            self.particle_cloud[i].pose.orientation = Quaternion(new_angle[0], new_angle[1], new_angle[2], new_angle[3]) 

            ang_diff = get_yaw_from_pose(j.pose) - get_yaw_from_pose(self.odom_pose_last_motion_update.pose)


            self.particle_cloud[i].pose.position.x = j.pose.position.x + v*math.cos(ang_diff) + np.random.normal(0,.1)
            self.particle_cloud[i].pose.position.y = j.pose.position.y + v*math.sin(ang_diff) + np.random.normal(0,.1)
                 


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









