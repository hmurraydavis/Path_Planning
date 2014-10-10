import time
from go_to_waypoints import Forebrain

forebrain=Forebrain()

def distBtwnPts(pt1, pt2):
	'''Calculates the distance between two plainer points with the distance between two points formula.'''
	return ( (pt2[1]-pt1[1])**2 + (pt2[0]-pt1[0])**2  )**.5 #distance between 2 points formula

def BuildList():
	number_of_modes=int(raw_input('How many points would you like to have in your mission?'))
	mission=[]
	
	for mode in range (number_of_modes):
        pt=raw_input('GPS coordinates of point (in parenthesis:())? ')
        mission.append(['go to point',pt])
	print 'Mission is:', mission
	
def missionMannager(mission): #mission is a list of dictionaries
	print mission
	for mis in mission:
		print 'mission current ',mis
		if mis['mode']=='go to point':
			#TODO:Give it the robot's current position
			currentPos=(2,3)
			while distBtwnPts(currentPos, mis['pt']) >= 0.2:
				forebrain.goToPoint(mis['pt'])
				time.sleep(.25)
missionT=[
	{'mode':'Line follow',
	'pt1':[4.5,8.99],
	'pt2':[7.85,8.95]},
	{'mode':'Obstacle avoid',
	'time':10},
	{'mode':'go to point',
	'pt':[9,10]},
	{'mode':'Maintain heading',
	'heading':70,
	'time':20}
]

mis=[

    ]
missionMannager(missionT)
