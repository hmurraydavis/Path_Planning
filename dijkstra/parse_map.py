#PARSE THE CSV, BINARY (0 OR 1), *.TXT MAP FILE:

import csv

map_path='/home/comprobo/Path_Planning/dijkstra/map2ex.txt'

#map = open(map_path, 'r')

#Read in from CSV file:
with open(map_path, 'Ur') as f:
    data = list(tuple(rec) for rec in csv.reader(f, delimiter=','))

#parse the resulting thing
for y, x_items in enumerate(data):
    for x, map_value_for_square in enumerate(x_items):
        if (map_value_for_square>0): #1 is traverseable, 0 is not
            neighbors=[]
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
                         neighbors.append((xn,yn))
