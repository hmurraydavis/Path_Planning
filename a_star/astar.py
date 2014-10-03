#PARSE THE CSV, BINARY (0 OR 1), *.TXT MAP FILE:

import csv

map_path='/home/jasper/comprobo2014/Path_Planning/a_star/map1ex.txt'

class Data():
    def __init__(self):
        self.closedset = []
        self.openset = []
        came_from = []
        heuristic_cost_estimate = 0 #each box has one
        g_score = 0
        f_score = g_score + heuristic_cost_estimate 

        with open(map_path, 'Ur') as f:
            self.data = list(tuple(rec) for rec in csv.reader(f, delimiter=','))

    def main(self):
        for y, x_items in enumerate(self.data):
            print y,x_items
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
                    yn, xn=neighbor #should be: xn,yn
                    if ((xn>=0) and (yn>=0) and (yn<len(x_items)) and (xn<len(self.data))):
                        pass
                        #print (xn, yn)
                        if (self.data[xn][yn]>0): #1 is traversable, 0 is not
                            neighbors.append((xn,yn))
                    #print 'neighbors of (',x,',',y,') are: ',neighbors
        #	map={tuple(x,y):
        return self

if __name__ == '__main__':
    d = Data()
    d.main()
