#PARSE THE CSV, BINARY (0 OR 1), *.TXT MAP FILE:

map_path='/home/comprobo/Path_Planning/dijkstra/map1ex.txt'

map = open(map_path, 'r')

print map.read()
