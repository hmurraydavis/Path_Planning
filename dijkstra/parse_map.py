#PARSE THE CSV, BINARY (0 OR 1), *.TXT MAP FILE:

import csv

map_path='/home/comprobo/Path_Planning/dijkstra/map1ex.txt'

#map = open(map_path, 'r')

with open(map_path, 'Ur') as f:
    data = list(tuple(rec) for rec in csv.reader(f, delimiter=','))

print 'type is: ', type(int(data[1][0]))

for x in data:
    print 'x is: ',len(x)
    curent_row=x
    for y in x:
        print 'hi'
#    print data[x]


#map.close()
