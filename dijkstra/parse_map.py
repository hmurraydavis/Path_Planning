#PARSE THE CSV, BINARY (0 OR 1), *.TXT MAP FILE:

import csv

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

data = read_csv('map2ex.txt')

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



