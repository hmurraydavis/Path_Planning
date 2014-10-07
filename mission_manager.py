import time
import bank
from Forebrain import Forebrain

forebrain=Forebrain()

def distBtwnPts(pt1, pt2):
	'''Calculates the distance between two plainer points with the distance between two points formula.'''
	return ( (pt2[1]-pt1[1])**2 + (pt2[0]-pt1[0])**2  )**.5 #distance between 2 points formula

def BuildList():
	
	number_of_modes=int(raw_input('How many modes would you like to have in your mission?'))
	
	mission=[]
	
	for mode in range (number_of_modes):
		mode=raw_input('What mode would you like to use here? choces: "line follow," "obstacle avoid," "go to point," "maintain heading" ')
		if mode=='line follow':
			pt1=raw_input('GPS Coordinates of first point (in brackets:[])? ')
			pt2=raw_input('GPS Coordinates of second point (in brackets:[])? ')
			mission.append(['line follow',pt1,pt2])
			
		if mode=='obstacle avoid':
			timeT=raw_input('How long would you like it to avoid obstacles on its own?')
			mission.append(['obstacle avoid',timeT])
			
		if mode=='go to point':
			pt=raw_input('GPS coordinates of point (in brackets:[])? ')
			mission.append(['go to point',pt])
			
		if mode=='maintain heading':
			heading=raw_input('Desired global heading (in degrees)?')
			timeT=timeT=raw_input('How long would you like it to maintain this heading?')
			mission.append(['maintain heading',heading,timeT])
			
	print 'Mission is:', mission
	
	#TODO: make the mission info not be a string ex: [['linefollow'], [2,3],[4,5]] all 1 string == sad
	
def missionMannager(mission): #mission is a list of dictionaries
	print mission
	for mis in mission:
		print 'mission current ',mis
		if mis['mode']=='line follow':
			pt1 = mis['pt1']
			print 'pt1 is:', pt1
			pt2 = mis['pt2']
			currentPos=forebrain.readPosition()
			distToPt=distBtwnPts(currentPos, mis['pt2'])
			print 'Dist between pts: ',distToPt
			while distToPt>=0.00002:
				forebrain.followLine(pt1, pt2) #followLine(self,posCurrent,linstart,posDesired
				time.sleep(.25)
		elif mis['mode']=='obstacle avoid':
			for t in range (0,2*mis['time']):
				forebrain.obsAvoid()
		elif mis['mode']=='go to point':
			currentPos=forebrain.readPosition()
			while distBtwnPts(currentPos, mis['pt']) >= 0.00002:
				forebrain.goToPoint(mis['pt'])
				time.sleep(.25)
		elif mis['mode']=='maintain heading':
			for t in range (0,4*int(mis['time'])):
				forebrain.mtnHeading(mis['heading'])
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
missionMannager(missionT)
