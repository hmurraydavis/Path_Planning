import math
#import bank #delete when final

class Forebrain:	
	def __init__(self):
                #init things here. Now depricated
                pass
	
	## Going to ppoints!
	def slope(self,p2,p1):
		# '''gives the slope of the line to be followed
		#tested to work 12:45 PM, 6/11/14 '''
		m=float(p2[1]-p1[1])/float(p2[0]-p1[0])
		#print 'Slope is: ' + str(m)
		return m

	def headingToPoint(self,posDesired):
		#'''gives the heading (angle in degrees from the +x axis) of the line to be followed'''

		#TODO: give robot its current position:
		posCurrent = (2,3) #robot's current position
		m=self.slope(posDesired,posCurrent)
		angle=math.degrees(math.atan(m/1))

		xcheck=(posDesired[0]-posCurrent[0])>0#if true, in I or IV
		ycheck=(posDesired[1]-posCurrent[1])>0#if true, in I or II
		#print 'x check: ' +str(xcheck) + '. y check:' +str(ycheck)+'.'

		if (ycheck==True) & (xcheck==False):  #quadrent 2
			angle=90+math.fabs(angle)
			#print 'Boat should be traveling into quadrent 2!'

		if  (xcheck==True) & (ycheck==False): #quadrent 4
			angle=270+math.fabs(angle)
			#print 'Boat should be heading into quadrent 4!'

		if (xcheck==False) & (ycheck==False): #quadrent 3
			angle =180+math.fabs(angle)
			#print 'Boat should be heading into quadrent 3!'
		#if quadrent 1, the angle doesn't need to be parsed.

		return angle

	def goToPoint(self,pt):
		# '''outputs the necessary, global heading in order to go toward a point'''

		currentHeading = self.midbrain.readHeading()
		headingtoPoint=self.headingToPoint(pt)
		print 'currentHeading' +str(currentHeading)
		print 'headingtoPoint' +str(headingtoPoint)
	
		desiredHeading=float(currentHeading+headingtoPoint)/2
		print 'the desire heading is:'+str(desiredHeading)
		return desiredHeading
		
		
	## Following lines!
	#def slope(self,posDesired,linstart):
		# '''gives the slope of the line to be followed
		#tested to work 12:45 PM, 6/11/14'''
	#	m=float(posDesired[1]-linstart[1])/float(posDesired[0]-linstart[0])
		#print 'Slope is: ' + str(m)
	#	return m

	def heading_line(self,linstart,posDesired):
		#'''gives the heading (angle in degrees from the +x axis) of the line to be followed'''

		m=self.slope(posDesired,linstart)
		angle=math.degrees(math.atan(m/1))

		xcheck=(posDesired[0]-linstart[0])>0#if true, in I or IV
		ycheck=(posDesired[1]-linstart[1])>0#if true, in I or II
		#print 'x check: ' +str(xcheck) + '. y check:' +str(ycheck)+'.'

		if (ycheck==True) & (xcheck==False):  #quadrent 2
			angle=90+math.fabs(angle)
			#print 'Boat should be traveling into quadrent 2!'

		if  (xcheck==True) & (ycheck==False): #quadrent 4
			angle=270+math.fabs(angle)
			#print 'Boat should be heading into quadrent 4!'

		if (xcheck==False) & (ycheck==False): #quadrent 3
			angle =180+math.fabs(angle)
			#print 'Boat should be heading into quadrent 3!'
		#if quadrent 1, the angle doesn't need to be parsed.

		return angle


	def above_below_on_line(self,posCurrent,linstart,posDesired):
		# '''gives whether the bot is above, below, or on the line it should be following.
		#tested to work at 13:14 on 6/11/14
		# Based on: http://math.stackexchange.com/questions/324589/detecting-whether-a-point-is-above-or-below-a-slope'''

		#TODO: give the robot it's current position
		posCurrent = (2,3)
		m=self.slope(posDesired, linstart)
		b=posDesired[1]-(m*posDesired[0])
		check=m*posCurrent[0]+b
		if check<posCurrent[1]:
			print 'Bot is above the line'
			return 'above'
		if check>posCurrent[1]:
			print 'Bot is below the line'
			return 'below'
		if check == posCurrent[1]:
			print 'Bot is on the line!'
			return 'on'
	

	def followLine(self,linstart,posDesired):
		# '''Outputs the necessary, global heading for the robot to follow a line with specified endpoints'''
		
		#TODO: give it the robot's position
		posCurrent=(2,3)
		bot_posVlin=self.above_below_on_line(posCurrent,linstart,posDesired)
		lineheading=self.heading_line(linstart,posDesired)
		#TODO: Read in the current heading of the robot
		botheading = 45
		print 'Line heading: '+str(lineheading)
		print 'Bot heading: '+str(botheading)
		print 'bot_posVlin', bot_posVlin
		
		#check through possible cases and assign the correct, desired, global heading accordingly
		if (bot_posVlin=='above') & (botheading>lineheading): #above line and heading away from it.
			print 'Bot above and heading away from line to be followed'
			headDesired=((lineheading-botheading)/2)+lineheading

		elif (bot_posVlin=='below') & (botheading>lineheading): #below line and heading toward it.
			print 'Bot below and heading away from line to be followed'
			headDesired=((lineheading-botheading)/2)+lineheading

		elif (bot_posVlin=='below') & (botheading<lineheading): #below line and heading away from it. 
			print 'Bot below and heading toward from line to be followed'
			headDesired=lineheading-math.fabs((lineheading-botheading)/2)

		elif (bot_posVlin=='above') & (botheading<lineheading): #above line and heading toward it.
			print 'Bot above and heading toward from line to be followed'
			headDesired=lineheading-math.fabs((lineheading-botheading)/2)
		elif bot_posVlin=='on': #on line! :)
			print 'Bot is on the line to be followed!'
			headDesired=lineheading
			
			
		print 'The desired heading for the bot is:' + str(headDesired)
		return headDesired
		
	
if __name__=='__main__':
	FB=Forebrain()
	FB.followLine([3,4],[1,0]) #followLine(self,posCurrent,linstart,posDesired):
	FB.above_below_on_line([3,4],[1,0],[9,10])
	FB.heading_line([1,0],[9,10]) #heading_line(self,linstart,posDesired)
	FB.headingToPoint([9,10])
