"""goes to point published by Halie's function that directs it go to a waypoint"""
"""subscribes to Halie's function, which publishes waypoints"""

#initialize
#get map from Halie's function
#try to match it with the SLAM + particle filter

class Point():
	#x_dist, y_dist, flagged attributes

def read_waypoints():
	#import points
	#get list of points	
	#loop through points and create a list of objects with x_dist, y_dist, flagged attributes
	#return list of Point objects

class Direction():
	#object target Point
	#object pf Point
	#object target angle
	#object pf angle

	def __init__():
		# one time thing
		# pass in a map of x by x units
		# compare discrete map with slam algorithm

	def list_of_points():
		#pass in order of objects
		#checks out the various points
		#return current waypoints

	def where_am_i():
		#always running
		#return pf point pt

	def am_i_close():
		#always running
		#check pf point close to waypoint
		#return target point

	def okay_lets_go():
		#compares target to pf pt
		#talk to go_to_waypoints
		#publish cmd_vel