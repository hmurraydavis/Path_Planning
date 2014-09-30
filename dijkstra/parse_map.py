#PARSE THE CSV, BINARY (0 OR 1), *.TXT MAP FILE:

map_path='/home/comprobo/Path_Planning/dijkstra/map1ex.txt'

#map = open(map_path, 'r')

#maptxt= map.read()

#print type(maptxt)

import csv

with open(map_path, 'Ur') as f:
    data = list(tuple(rec) for rec in csv.reader(f, delimiter=','))

print data
print type(data)
print 'tp 1=: ', data[1][0]
print type(int(data[1][0]))

#map.close()
