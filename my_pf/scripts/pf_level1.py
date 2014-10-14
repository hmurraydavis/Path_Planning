#!/usr/bin/env python

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors

class TransformHelpers:
	""" Some convenience functions for translating between various representions of a robot pose.
		TODO: nothing... you should not have to modify these """

	@staticmethod
	def convert_translation_rotation_to_pose(translation, rotation):
		""" Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
		return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))

	@staticmethod
	def convert_pose_inverse_transform(pose):
		""" Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) """
		translation = np.zeros((4,1))
		translation[0] = -pose.position.x
		translation[1] = -pose.position.y
		translation[2] = -pose.position.z
		translation[3] = 1.0

		rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		euler_angle = euler_from_quaternion(rotation)
		rotation = np.transpose(rotation_matrix(euler_angle[2], [0,0,1]))		# the angle is a yaw
		transformed_translation = rotation.dot(translation)

		translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
		rotation = quaternion_from_matrix(rotation)
		return (translation, rotation)

	@staticmethod
	def convert_pose_to_xy_and_theta(pose):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		return (pose.position.x, pose.position.y, angles[2])

class Particle:
	""" Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
		Attributes:
			x: the x-coordinate of the hypothesis relative to the map frame
			y: the y-coordinate of the hypothesis relative ot the map frame
			theta: the yaw of the hypothesis relative to the map frame
			w: the particle weight (the class does not ensure that particle weights are normalized
	"""

	def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
		""" Construct a new Particle
			x: the x-coordinate of the hypothesis relative to the map frame
			y: the y-coordinate of the hypothesis relative ot the map frame
			theta: the yaw of the hypothesis relative to the map frame
			w: the particle weight (the class does not ensure that particle weights are normalized """ 
		self.w = w
		self.theta = theta
		self.x = x
		self.y = y

	def as_pose(self):
		""" A helper function to convert a particle to a geometry_msgs/Pose message """
		orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
		return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))

	# TODO: define additional helper functions if needed

class OccupancyField:
	""" Stores an occupancy field for an input map.  An occupancy field returns the distance to the closest
		obstacle for any coordinate in the map
		Attributes:
			map: the map to localize against (nav_msgs/OccupancyGrid)
			closest_occ: the distance for each entry in the OccupancyGrid to the closest obstacle
	"""

	def __init__(self, map):
		self.map = map		# save this for later
		# build up a numpy array of the coordinates of each grid cell in the map
		X = np.zeros((self.map.info.width*self.map.info.height,2))

		# while we're at it let's count the number of occupied cells
		total_occupied = 0
		curr = 0
		for i in range(self.map.info.width):
			for j in range(self.map.info.height):
				# occupancy grids are stored in row major order, if you go through this right, you might be able to use curr
				ind = i + j*self.map.info.width
				if self.map.data[ind] > 0:
					total_occupied += 1
				X[curr,0] = float(i)
				X[curr,1] = float(j)
				curr += 1

		# build up a numpy array of the coordinates of each occupied grid cell in the map
		O = np.zeros((total_occupied,2))
		curr = 0
		for i in range(self.map.info.width):
			for j in range(self.map.info.height):
				# occupancy grids are stored in row major order, if you go through this right, you might be able to use curr
				ind = i + j*self.map.info.width
				if self.map.data[ind] > 0:
					O[curr,0] = float(i)
					O[curr,1] = float(j)
					curr += 1

		# use super fast scikit learn nearest neighbor algorithm
		nbrs = NearestNeighbors(n_neighbors=1,algorithm="ball_tree").fit(O)
		distances, indices = nbrs.kneighbors(X)

		self.closest_occ = {}
		curr = 0
		for i in range(self.map.info.width):
			for j in range(self.map.info.height):
				ind = i + j*self.map.info.width
				self.closest_occ[ind] = distances[curr][0]*self.map.info.resolution
				curr += 1

	def get_closest_obstacle_distance(self,x,y):
		""" Compute the closest obstacle to the specified (x,y) coordinate in the map.  If the (x,y) coordinate
			is out of the map boundaries, nan will be returned. """
		x_coord = int((x - self.map.info.origin.position.x)/self.map.info.resolution)
		y_coord = int((y - self.map.info.origin.position.y)/self.map.info.resolution)

		# check if we are in bounds
		if x_coord > self.map.info.width or x_coord < 0:
			return float('nan')
		if y_coord > self.map.info.height or y_coord < 0:
			return float('nan')

		ind = x_coord + y_coord*self.map.info.width
		if ind >= self.map.info.width*self.map.info.height or ind < 0:
			return float('nan')
		return self.closest_occ[ind]

class ParticleFilter:
	""" The class that represents a Particle Filter ROS Node
		Attributes list:
			initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
			base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
			map_frame: the name of the map coordinate frame (should be "map" in most cases)
			odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
			scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
			n_particles: the number of particles in the filter
			d_thresh: the amount of linear movement before triggering a filter update
			a_thresh: the amount of angular movement before triggering a filter update
			laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
			pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
			particle_pub: a publisher for the particle cloud
			laser_subscriber: listens for new scan data on topic self.scan_topic
			tf_listener: listener for coordinate transforms
			tf_broadcaster: broadcaster for coordinate transforms
			particle_cloud: a list of particles representing a probability distribution over robot poses
			current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
								   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
			map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
	"""
	def __init__(self):
		self.initialized = False		# make sure we don't perform updates before everything is setup
		rospy.init_node('pf')			# tell roscore that we are creating a new node named "pf"

		self.base_frame = "base_link"	# the frame of the robot base
		self.map_frame = "map"			# the name of the map coordinate frame
		self.odom_frame = "odom"		# the name of the odometry coordinate frame
		self.scan_topic = "scan"		# the topic where we will get laser scans from 

		self.n_particles = 300			# the number of particles to use

		self.d_thresh = 0.2				# the amount of linear movement before performing an update
		self.a_thresh = math.pi/6		# the amount of angular movement before performing an update

		self.laser_max_distance = 2.0	# maximum penalty to assess in the likelihood field model

		# TODO: define additional constants if needed

		# Setup pubs and subs

		# pose_listener responds to selection of a new approximate robot location (for instance using rviz)
		self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
		# publish the current particle cloud.  This enables viewing particles in rviz.
		self.particle_pub = rospy.Publisher("particlecloud", PoseArray)

		# laser_subscriber listens for data from the lidar
		self.laser_subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received)

		# enable listening for and broadcasting coordinate transforms
		self.tf_listener = TransformListener()
		self.tf_broadcaster = TransformBroadcaster()

		self.particle_cloud = []

		self.current_odom_xy_theta = []

		# request the map from the map server, the map should be of type nav_msgs/OccupancyGrid
		# TODO: fill in the appropriate service call here.  The resultant map should be assigned be passed
		#		into the init method for OccupancyField

		# for now we have commented out the occupancy field initialization until you can successfully fetch the map
		#self.occupancy_field = OccupancyField(map)
		self.initialized = True

	def update_robot_pose(self):
		""" Update the estimate of the robot's pose given the updated particles.
			There are two logical methods for this:
				(1): compute the mean pose (level 2)
				(2): compute the most likely pose (i.e. the mode of the distribution) (level 1)
		"""
		# first make sure that the particle weights are normalized
		self.normalize_particles()

		# TODO: assign the lastest pose into self.robot_pose as a geometry_msgs.Pose object
		# just to get started we will fix the robot's pose to always be at the origin
		self.robot_pose = Pose()

	def update_particles_with_odom(self, msg):
		""" Update the particles using the newly given odometry pose.
			The function computes the value delta which is a tuple (x,y,theta)
			that indicates the change in position and angle between the odometry
			when the particles were last updated and the current odometry.

			msg: this is not really needed to implement this, but is here just in case.
		"""
		new_odom_xy_theta = TransformHelpers.convert_pose_to_xy_and_theta(self.odom_pose.pose)
		# compute the change in x,y,theta since our last update
		if self.current_odom_xy_theta:
			old_odom_xy_theta = self.current_odom_xy_theta
			delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0], new_odom_xy_theta[1] - self.current_odom_xy_theta[1], new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
			self.current_odom_xy_theta = new_odom_xy_theta
		else:
			self.current_odom_xy_theta = new_odom_xy_theta
			return

		# TODO: modify particles using delta
		# For added difficulty: Implement sample_motion_odometry (Prob Rob p 136)

	def map_calc_range(self,x,y,theta):
		""" Difficulty Level 3: implement a ray tracing likelihood model... Let me know if you are interested """
		# TODO: nothing unless you want to try this alternate likelihood model
		pass

	def resample_particles(self):
		""" Resample the particles according to the new particle weights.
			The weights stored with each particle should define the probability that a particular
			particle is selected in the resampling step.  You may want to make use of the given helper
			function draw_random_sample.
		"""
		# make sure the distribution is normalized
		self.normalize_particles()
		# TODO: fill out the rest of the implementation

	def update_particles_with_laser(self, msg):
		""" Updates the particle weights in response to the scan contained in the msg """
		# TODO: implement this
		pass

	@staticmethod
	def angle_normalize(z):
		""" convenience function to map an angle to the range [-pi,pi] """
		return math.atan2(math.sin(z), math.cos(z))

	@staticmethod
	def angle_diff(a, b):
		""" Calculates the difference between angle a and angle b (both should be in radians)
			the difference is always based on the closest rotation from angle a to angle b
			examples:
				angle_diff(.1,.2) -> -.1
				angle_diff(.1, 2*math.pi - .1) -> .2
				angle_diff(.1, .2+2*math.pi) -> -.1
		"""
		a = ParticleFilter.angle_normalize(a)
		b = ParticleFilter.angle_normalize(b)
		d1 = a-b
		d2 = 2*math.pi - math.fabs(d1)
		if d1 > 0:
			d2 *= -1.0
		if math.fabs(d1) < math.fabs(d2):
			return d1
		else:
			return d2

	@staticmethod
	def weighted_values(values, probabilities, size):
		""" Return a random sample of size elements from the set values with the specified probabilities
			values: the values to sample from (numpy.ndarray)
			probabilities: the probability of selecting each element in values (numpy.ndarray)
			size: the number of samples
		"""
		bins = np.add.accumulate(probabilities)
		return values[np.digitize(random_sample(size), bins)]

	@staticmethod
	def draw_random_sample(choices, probabilities, n):
		""" Return a random sample of n elements from the set choices with the specified probabilities
			choices: the values to sample from represented as a list
			probabilities: the probability of selecting each element in choices represented as a list
			n: the number of samples
		"""
		values = np.array(range(len(choices)))
		probs = np.array(probabilities)
		bins = np.add.accumulate(probs)
		inds = values[np.digitize(random_sample(n), bins)]
		samples = []
		for i in inds:
			samples.append(deepcopy(choices[int(i)]))
		return samples

	def update_initial_pose(self, msg):
		""" Callback function to handle re-initializing the particle filter based on a pose estimate.
			These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
		xy_theta = TransformHelpers.convert_pose_to_xy_and_theta(msg.pose.pose)
		self.initialize_particle_cloud(xy_theta)
		self.fix_map_to_odom_transform(msg)

	def initialize_particle_cloud(self, xy_theta=None):
		""" Initialize the particle cloud.
			Arguments
			xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
					  particle cloud around.  If this input is ommitted, the odometry will be used """
		if xy_theta == None:
			xy_theta = TransformHelpers.convert_pose_to_xy_and_theta(self.odom_pose.pose)
		self.particle_cloud = []
		self.particle_cloud.append(Particle(0,0,0))
		# TODO create particles

		self.normalize_particles()
		self.update_robot_pose()

	def normalize_particles(self):
		""" Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
		# TODO: implement this

	def publish_particles(self, msg):
		particles_conv = []
		for p in self.particle_cloud:
			particles_conv.append(p.as_pose())
		# actually send the message so that we can view it in rviz
		self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),frame_id=self.map_frame),poses=particles_conv))

	def scan_received(self, msg):
		""" This is the default logic for what to do when processing scan data.  Feel free to modify this, however,
			I hope it will provide a good guide.  The input msg is an object of type sensor_msgs/LaserScan """
		if not(self.initialized):
			# wait for initialization to complete
			return

		if not(self.tf_listener.canTransform(self.base_frame,msg.header.frame_id,msg.header.stamp)):
			# need to know how to transform the laser to the base frame
			# this will be given by either Gazebo or neato_node
			return

		if not(self.tf_listener.canTransform(self.base_frame,self.odom_frame,msg.header.stamp)):
			# need to know how to transform between base and odometric frames
			# this will eventually be published by either Gazebo or neato_node
			return

		# calculate pose of laser relative ot the robot base
		p = PoseStamped(header=Header(stamp=rospy.Time(0),frame_id=msg.header.frame_id))
		self.laser_pose = self.tf_listener.transformPose(self.base_frame,p)

		# find out where the robot thinks it is based on its odometry
		p = PoseStamped(header=Header(stamp=msg.header.stamp,frame_id=self.base_frame), pose=Pose())
		self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
		# store the the odometry pose in a more convenient format (x,y,theta)
		new_odom_xy_theta = TransformHelpers.convert_pose_to_xy_and_theta(self.odom_pose.pose)

		if not(self.particle_cloud):
			# now that we have all of the necessary transforms we can update the particle cloud
			self.initialize_particle_cloud()
			# cache the last odometric pose so we can only update our particle filter if we move more than self.d_thresh or self.a_thresh
			self.current_odom_xy_theta = new_odom_xy_theta
			# update our map to odom transform now that the particles are initialized
			self.fix_map_to_odom_transform(msg)
		elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or
			  math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or
			  math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh):
			# we have moved far enough to do an update!
			self.update_particles_with_odom(msg)	# update based on odometry
			self.update_particles_with_laser(msg)	# update based on laser scan
			self.update_robot_pose()				# update robot's pose
			self.resample_particles()				# resample particles to focus on areas of high density
			self.fix_map_to_odom_transform(msg)		# update map to odom transform now that we have new particles
		# publish particles (so things like rviz can see them)
		self.publish_particles(msg)

	def fix_map_to_odom_transform(self, msg):
		""" Super tricky code to properly update map to odom transform... do not modify this... Difficulty level infinity. """
		(translation, rotation) = TransformHelpers.convert_pose_inverse_transform(self.robot_pose)
		p = PoseStamped(pose=TransformHelpers.convert_translation_rotation_to_pose(translation,rotation),header=Header(stamp=msg.header.stamp,frame_id=self.base_frame))
		self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
		(self.translation, self.rotation) = TransformHelpers.convert_pose_inverse_transform(self.odom_to_map.pose)

	def broadcast_last_transform(self):
		""" Make sure that we are always broadcasting the last map to odom transformation.
			This is necessary so things like move_base can work properly. """
		if not(hasattr(self,'translation') and hasattr(self,'rotation')):
			return
		self.tf_broadcaster.sendTransform(self.translation, self.rotation, rospy.get_rostime(), self.odom_frame, self.map_frame)

if __name__ == '__main__':
	n = ParticleFilter()
	r = rospy.Rate(5)

	while not(rospy.is_shutdown()):
		# in the main loop all we do is continuously broadcast the latest map to odom transform
		n.broadcast_last_transform()
		r.sleep()
