#!/usr/bin/env python

#PARSE THE CSV, BINARY (0 OR 1), *.TXT MAP FILE:
import rospy
import numpy
import csv
import pprint
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import OccupancyGrid

global pub
d = 0

def read_csv(map_path):
    '''Opens and reads in CSV files. These represent the map of the
        world for the robot. 
        1 = traversable
        0 = not traversable 
        
        INPUT: file path or file name of the desired map
        OUTPUT: list of tuples of ints (effectively a 2d array) of the map '''
    #Read in from CSV file:
    with open(map_path) as f:
        return list(tuple(rec) for rec in csv.reader(f, delimiter=','))

def distance_between_neighbors(pt1,pt2):
    '''Compute the distance between two points. 
       INPUT: pt1 = tuple representing the first point
              pt2 = tuple representing rhe second point
       OUTPUT: float which is the distance between the two, given points'''
    return ( (pt2[1]-pt1[1])**2 + (pt2[0]-pt1[0])**2 )**.5


def make_space_dict():
	data = read_csv('map1ex.txt')

	space={} #initialize the dictionary, representing thetraverable space of nodes

	#parse the resulting list of tuples of ints (effectively a 2d array)
	for y, x_items in enumerate(data):
	    for x, map_value_for_square in enumerate(x_items):
		if (map_value_for_square>0): #1 is traverseable, 0 is not
		    neighbors={}
		    potential_neighbors=[(x+1,y),
					 (x-1,y),
					 (x,y+1),
					 (x,y-1),
					 (x+1,y+1),
					 (x-1,y-1),
					 (x+1,y-1),
					 (x-1,y+1)]
		    for neighbor in potential_neighbors:
			xn, yn = neighbor
			if ((xn>=0) and (yn>=0) 
			     and (xn<len(x_items)) and (yn<len(data))): #check if it's in bounds of the map
			     if data[yn][xn]>0: #check if it's traversable. Data is read in backwards, so data[yn][xn] is correct.
				 neighbors[(xn,yn)]=distance_between_neighbors((x,y),(xn,yn))
				 
		    #construct dictionary of point and its neighbors!
		    space[(x,y)] = neighbors
	return space

def dijkstra(origin, goal,pub):
    space=make_space_dict() #make map from file
    nodesVisited=set((origin,))
    node_dists={origin:0}
    node_progressions={}
    return dijkstraR(space, origin, goal, nodesVisited, node_dists, node_progressions,pub) 

def dijkstraR(space, currentNode, goal, nodesVisited, node_dists, node_progressions,pub):
    if currentNode == goal:
        node_path = []
        end_of_path = currentNode
        while end_of_path != False:
            node_path.append(end_of_path)
            end_of_path = node_progressions.get(end_of_path, False)

        print 'you are winnerr'
        desired_path=list(reversed(node_path))
        pub.publish(desired_path)
        return desired_path

    for child in space[currentNode]:
        if child not in nodesVisited: #if the edge hasn't been checked, check it
            #if child == goal: return
            checkDist=node_dists[currentNode]+space[currentNode][child]
            if node_dists.get(child, float('inf')) > checkDist: #update path if shorter route
                node_dists[child] = checkDist
                node_progressions[child]=currentNode
    nodesVisited.add(currentNode)

    traverseNext = sorted(node_dists.items(), key=node_dists.get)
    #print '\n To traverse:', pprint.pprint(traverseNext)

    sorted_nodes = [node for node, _ in traverseNext if node not in nodesVisited]
    if not len(sorted_nodes):
        print 'no more nodes to visit'
        return
    else:
        closest_node = sorted_nodes[0]
        print 'closest:', closest_node

    #Recurse over the children search front, starting with the current shortest path
    print 'closest node is: ',closest_node
    return dijkstraR(space, closest_node, goal, nodesVisited, node_dists, node_progressions,pub)

#print dijkstra((0,0), (0,4))

def read_in_map(msg):
    """ Processes data from the laser scanner and makes it available to other functions
    INPUT: The data from a single laser scan_received
    OUTPUT: 
    **Writes laser scan data to the global variable: lazer_measurements"""

    global mapSpace
    pprint.pprint(msg) #TODO: Do something with the message map gotten from jasper's code
    nparray = np.array(msg)
    nparray.resize(512,512)
    #print msg
    mapSpace=nparray
    #print mapSpace


if __name__ == '__main__':
    '''Initializes ROS processes and controls the state of the robot once 
    indivigual behaviors yield controls
    INPUT: none
    OUTPUT: none'''
    try:
        rospy.init_node('test', anonymous=True)
        #global pub
        pub = rospy.Publisher('waypoint_list', Int16MultiArray)
        sub = rospy.Subscriber('map', OccupancyGrid, read_in_map) #TODO: change topic to be that of the map
     #   dijkstra((0,0),(1,1),pub) #TODO: make it the actual goal and starting location
    except rospy.ROSInterruptException: pass
