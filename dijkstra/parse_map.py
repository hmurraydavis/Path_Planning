#PARSE THE CSV, BINARY (0 OR 1), *.TXT MAP FILE:

import csv

map_path='/home/comprobo/Path_Planning/dijkstra/map2ex.txt'

#map = open(map_path, 'r')

#Read in from CSV file:
with open(map_path, 'Ur') as f:
    data = list(tuple(rec) for rec in csv.reader(f, delimiter=','))

#parse the resulting thin:g
for x, x_items in enumerate(data):
    for y, location in enumerate(x_items):
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
            xn,yn=neighbor
            if ((xn>=0) and (yn>=0) and (xn<=len(x_items)) and (yn<=len(data))):
                if (data[xn][yn]>0): #1 is traversable, 0 is not
                    neighbors.append((xn,yn))
            print 'neighbors of (',x,',',y,') are: ',neighbors
#	map={tuple(x,y):
