#PARSE THE CSV, BINARY (0 OR 1), *.TXT MAP FILE:

map_path='/home/comprobo/Path_Planning/dijkstra/map1ex.txt'

#map = open(map_path, 'r')

#maptxt= map.read()

#print type(maptxt)

import csv

with open(map_path, 'Ur') as f:
    data = list(tuple(rec) for rec in csv.reader(f, delimiter=','))

print 'type is: ', type(int(data[1][0]))

for x in data:
    for y in len(x):
        print x[y]
#    print data[x]


#map.close()
