
import anki_vector
from anki_vector.events import Events
import cv2 as cv
import numpy as np
import time

# Constants

Debug = False

StartPositionRange = {'min':-5.0, 'max':5.0}
StartAngleRange = {'min':-5.0, 'max':5.0}
StartAttemptsMax = 3

# milliseconds per main loop execution
MainLoopDelay = 20

class Goal:
	found = False
	pose = None

class Puck:
	HsvMin = (0, 50, 30)
	HsvMax = (15, 255, 255)
	RadiusMax = 178
	RadiusScale = 0.8455
	RadiusTolerance = 0.2
	AspectRatio = 3.0
	AspectRatioTolerance = 0.5
	AspectMin = (1 - AspectRatioTolerance) * AspectRatio
	AspectMax = (1 + AspectRatioTolerance) * AspectRatio
	TargetX = 320
	TargetY = 256

	found = False
	x = 0.0
	y = 0.0
	center = 0.0
	radius = 0.0
	deltaX = 0.0
	deltaY = 0.0
	imageId = 0


class Camera:
	FovH = 90.0
	FovV = 50.0
	PixelsH = 640.0
	PixelsV = 360.0
	DegressPerPixelH = FovH / PixelsH
	DegreesPerPixelV = FovV / PixelsV

DistancePerPixelV = 1.0
DriveSpeed = anki_vector.util.Speed(speed_mmps = 100.0)

HeadTilt = anki_vector.robot.MIN_HEAD_ANGLE + anki_vector.util.degrees(5.0)
DriveHeight = 0.0
GrabHeight =  50.0
GrabDistance = anki_vector.util.Distance(distance_mm = 5.0)
ClearDistance = anki_vector.util.Distance(distance_mm = -50.0)
GrabSpeed = anki_vector.util.Speed(speed_mmps = 20.0)
GrabToleranceMin = -10
GrabToleranceMax = 10

BehaviorMode = 0
MotorMode = 1

GoalTilt = anki_vector.robot.MIN_HEAD_ANGLE + anki_vector.util.degrees(25.0)
GoalOffsetX = 50.0
GoalOffsetY = 0.0
ScoreSpeed = anki_vector.util.Speed(speed_mmps = 100.0)


puck = Puck()
camera = Camera()
goal = Goal()


#
# I would expect Anki to define these somewhere in the SDK,
# but can't find them except for in a comment (???)
#
NoneRobotStatusFlag     = 0
IS_MOVING               = 0x1
IS_CARRYING_BLOCK       = 0x2
IS_PICKING_OR_PLACING   = 0x4
IS_PICKED_UP            = 0x8
IS_BUTTON_PRESSED       = 0x10
IS_FALLING              = 0x20
IS_ANIMATING            = 0x40
IS_PATHING              = 0x80
LIFT_IN_POS             = 0x100
HEAD_IN_POS             = 0x200
CALM_POWER_MODE         = 0x400
IS_BATTERY_DISCONNECTED = 0x800
IS_ON_CHARGER           = 0x1000
IS_CHARGING             = 0x2000
CLIFF_DETECTED          = 0x4000
ARE_WHEELS_MOVING       = 0x8000
IS_BEING_HELD           = 0x10000
IS_MOTION_DETECTED      = 0x20000
IS_BATTERY_OVERHEATED   = 0x40000

# Event handler for observed objects
def on_robot_observed_object(event, info):
	global goal
	goal.pose = info.pose
	goal.found = True

# find the puck in an image
def findPuck():
	global cvImage
	global cvImageId
	global puck

	# only do this if we have a new image
	if (cvImageId != puck.imageId):
		puck.imageId = cvImageId

		# Much information and some code was obtained from here:
		# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

		# blur, convert to HSV, look for the puck HSV values, do some filtering
		blurImage = cv.GaussianBlur(cvImage, (11, 11), 0)
		hsvImage = cv.cvtColor(blurImage, cv.COLOR_BGR2HSV)
		trackerImage = cv.inRange(hsvImage, puck.HsvMin, puck.HsvMax)
		trackerImage = cv.erode(trackerImage, None, iterations = 2)
		trackerImage = cv.dilate(trackerImage, None, iterations = 2)

		# find contours
		im2, contours, hierarchy = cv.findContours(trackerImage.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

		# We're given a a bunch of contours that might enclose the puck
		# To help decide if a contour is a puck:
		#   - pick the largest contour
		#   - find the radius of the enclosing circle (scale based on distance)
		#   - compute the aspect ratio
		# If the radius and aspect ratio are within the tolerances of a puck,
		# it most likely is a puck.  However, some features on the rink can
		# still come close to looking like a puck... more work could be done.

		if len(contours) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid		
			c = max(contours, key=cv.contourArea)
			((x, y), radius) = cv.minEnclosingCircle(c)
			M = cv.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# compute the aspect ratio
			bX, bY, bW, bH = cv.boundingRect(c)
			aspectRatio = float(bW) / float(bH)
	 
			deltaX = puck.TargetX - int(x) # positive is to the left
			deltaY = puck.TargetY - int(y) # positive is away

			# compute the radius we want to check agains based on distance in the Y
			# direction of the video frame
			radiusCheck = puck.RadiusMax - puck.RadiusScale * deltaY 

			# uncomment for debugging puck issues
			#print(f'[findPuck] ({x:.0f},{y:.0f}) ({deltaX:.0f},{deltaY:.0f}) R = {radius:.2f}')
			#print(f'[findPuck] width = {bW:.0f} height = {bH:.0f} aspect = {aspectRatio:.2f}')
			#print(f'[findPuck] radiusCheck = {radiusCheck:.2f}')

			# only proceed if the radius meets a certain size
			radiusMin = (1 - puck.RadiusTolerance) * radiusCheck
			radiusMax = (1 + puck.RadiusTolerance) * radiusCheck

			# perform the actual checks
			if ( (radiusMin < radius < radiusMax) and (puck.AspectMin < aspectRatio < puck.AspectMax) ):
				#print('[findPuck] Got it')
				puck.found = True
				puck.x = x
				puck.y = y
				puck.center = center
				puck.radius = radius
				puck.deltaX = deltaX
				puck.deltaY = deltaY

			else:
				puck.found = False


# Use the low level motor speed command to drive based on pixel deltas
# dX and dY are pixel distances away from zero
# This can be used to drive towards targets that we see via OpenCV
# i.e. the puck
# It is non-blocking and should be called as part of a control loop
def goToPixel(robot: anki_vector.Robot, dX: int, dY: int):
	global camera

	pH = 0.25
	pV = 0.5
	leftSpeed = (pV * dY) - (1.0 - abs(dY / camera.PixelsV)) * (pH * dX) 
	rightSpeed = (pV * dY) + (1.0 - abs(dY / camera.PixelsV)) * (pH * dX)
	#print(f'[goToPixel] {leftSpeed:.1f}:{rightSpeed:.1f}') 
	robot.motors.set_wheel_motors(leftSpeed, rightSpeed)


# We're going to make the crazy assumption that our charger is placed in the
# center of the goal on our side of the table.
# Before running this program, pick Vector up and then put him back down
# on the charger.  This will reset his pose to the charger location.
def setStartPosition(robot: anki_vector.Robot):
	goodStartPosition = False
	startAttempts = StartAttemptsMax
	while not(goodStartPosition) and (startAttempts > 0):
		pose = robot.pose
		if (StartPositionRange['min'] < pose.position.x < StartPositionRange['max'] and
			StartPositionRange['min'] < pose.position.y < StartPositionRange['max'] and
			StartAngleRange['min'] < pose.rotation.angle_z.degrees < StartAngleRange['max']):
			goodStartPosition = True
		else:
			print('[setStartPosition] Start position is out of tolerance')
			print('[setStartPosition] Pick Vector up and place him back down on the charger')
			time.sleep(5)
		startAttempts -= 1

	if not(goodStartPosition):
		print('[setStartPosition] Start position failure.')
		allDone(robot)

	print('[setStartPosition] Good start position')


def allDone(robot: anki_vector.Robot):
	print('[allDone] Cleaning up')
	robot.disconnect()
	cv.destroyAllWindows()
	exit()

class StateMachine:
	stateMachineTimer = 0
	lastPuckId = 0
	actionList = []
	ScanAngles = [0, -30, 30, -60, 60, -90, 90, -120, 120, -150, 150, 180]
	scanAngleIndex = 0

	def __init__(self, robot: anki_vector.Robot):
		self.robot = robot

	def actionsDone(self):
		done = True
		for i in self.actionList:
			if not(i.done()):
				done = False
		return done

	def action(self, b):
		self.actionList.append(b)

	# State machine that performs all the heavy logic
	def nextState(self, state: str):
		global puck
		global goal

		robot = self.robot
		nextState = 'Unknown'

		# check our motion status
		isMoving = (robot.status & IS_MOVING) == IS_MOVING
		isOnCharger = (robot.status & IS_ON_CHARGER) == IS_ON_CHARGER
		isPathing = (robot.status & IS_PATHING) == IS_PATHING

		if state == 'Start':
			# get off the charger
			self.action(robot.behavior.drive_off_charger())
			nextState = 'LeavingCharger'
		
		elif state == 'LeavingCharger':
			if self.actionsDone():				
				# make sure we can see properly
				self.action(robot.behavior.set_head_angle(HeadTilt))
				self.action(robot.behavior.set_lift_height(DriveHeight))
				# set the timer for 5 seconds
				# this gives the camera exposure time to settle
				self.stateMachineTimer = 5000 / MainLoopDelay
				nextState = 'CameraDelay'
			else:
				nextState = 'LeavingCharger'
		
		elif state == 'CameraDelay':
			self.stateMachineTimer -= 1
			if (self.stateMachineTimer <= 0):
				self.lastPuckId = 0
				# time has elapsed, camera should be ok
				self.stateMachineTimer = 1000 / MainLoopDelay
				nextState = 'LookForPuck'
			else:
				nextState = 'CameraDelay'

		elif state == 'LookForPuck':
			self.stateMachineTimer -= 1
			if puck.found:
				nextState = 'GoToPuck'
			elif (self.stateMachineTimer <= 0):
				nextState = 'ScanForPuck'
			else:
				nextState = 'LookForPuck'

		elif state == 'GoToPuck':
			if not(puck.found):
				nextState = 'LookForPuck'
			else:
				if (self.lastPuckId != puck.imageId):
					self.lastPuckId = puck.imageId

					if ( (GrabToleranceMin < puck.deltaX < GrabToleranceMax) and 
						 (GrabToleranceMin < puck.deltaY < GrabToleranceMax) ):
						# close enough!
						# stop the motors...
						robot.motors.set_wheel_motors(0.0, 0.0)
						nextState = 'GrabPuck'
					else:
						goToPixel(robot, puck.deltaX, puck.deltaY)
						nextState = 'GoToPuck'
				else:
					# stay here looking for an update
					nextState = 'GoToPuck'

		elif state == 'ScanForPuck':
			if puck.found:
				nextState = 'GoToPuck'
			else:
				if not(isMoving) and self.actionsDone():
					self.scanAngleIndex += 1
					if (self.scanAngleIndex == len(self.ScanAngles)):
						self.scanAngleIndex = 0
					self.action(robot.behavior.turn_in_place(angle=anki_vector.util.degrees(self.ScanAngles[self.scanAngleIndex]), 
						speed=anki_vector.util.degrees(60.0),
						is_absolute=True))
				nextState = 'ScanForPuck'

		elif state == 'GrabPuck':
			if isMoving:
				nextState = 'GrabPuck'
			else:
				if not(puck.found):
					nextState = 'LookForPuck'
				else:
					pose = anki_vector.util.Pose(x=1350.0, y=0.0, z=0.0, angle_z=anki_vector.util.Angle(degrees=0.0))
					self.action(robot.behavior.set_head_angle(GoalTilt))
					self.action(robot.behavior.go_to_pose(pose))
					nextState = 'HeadingDownRink'

		elif state == 'HeadingDownRink':
			if self.actionsDone():
				# clear the goal flag
				goal.found = False
				nextState = 'LookForGoal'
			else:
				nextState = 'HeadingDownRink'

		elif state == 'LookForGoal':
			if goal.found:
				scoreDistance = anki_vector.util.Distance(distance_mm = abs(goal.pose.x - robot.pose.position.x - GoalOffsetX))
				self.action(robot.behavior.drive_straight(scoreDistance, ScoreSpeed))
				nextState = 'Score'
			else:
				nextState = 'LookForGoal'

		elif state == 'Score':
			if self.actionsDone():
				nextState = 'BackAway'
			else:
				nextState = 'Score'

		elif state == 'BackAway':
			self.action(robot.behavior.drive_straight(ClearDistance, GrabSpeed))
			nextState = 'BackingAway'

		elif state == 'BackingAway':
			if self.actionsDone():
				nextState = 'GoHome'
			else:
				nextState = 'BackingAway'

		elif state == 'GoHome':
			pose = anki_vector.util.Pose(x=200.0, y=0.0, z=0.0, angle_z=anki_vector.util.Angle(degrees=0.0))
			self.action(robot.behavior.go_to_pose(pose))
			nextState = 'HeadingHome'

		elif state == 'HeadingHome':
			if self.actionsDone():
				nextState = 'Done'
			else:
				nextState = 'HeadingHome'

		else:
			# unknown state, let's be done
			print('[stateMachine] Unhandled state = ' + state)
			nextState = 'Done'

		return nextState

def main():	
	# this will be the OpenCV version of robot.camera.latest_image
	global cvImage
	global cvImageId
	global puck
	global camera
	global goal

	cvImageId = 0

	# open the video window
	cv.namedWindow('Vector', cv.WINDOW_NORMAL)

	# use AsyncRobot so that behaviors don't block
	robot = anki_vector.AsyncRobot(enable_camera_feed=True, default_logging=Debug)
	robot.connect()
	time.sleep(1)

	# subscribe to observed object events
	robot.events.subscribe(on_robot_observed_object, Events.robot_observed_object)

	stateMachine = StateMachine(robot)

	# make sure we're ready to start
	setStartPosition(robot)
	state = 'Start'
	lastState = 'Unknown'

	done = False
	displayImage = False

	while not(done):

		# check to see if we have a new image
		if (robot.camera.latest_image_id != cvImageId):
			# if we do, convert it to an OpenCV image
			# and keep track of the id
			pilImage = robot.camera.latest_image
			cvImageId = robot.camera.latest_image_id			
			cvImage = cv.cvtColor(np.array(pilImage), cv.COLOR_RGB2BGR)
			# display it
			displayImage = True

		# locate the puck (if we can see it)
		findPuck()

		# run the state machine
		if (state != lastState):
			print('[main] State = ' + state)

		lastState = state
		state = stateMachine.nextState(state)

		# check to see if the state machine is done
		if (state == 'Done'):
			robot.motors.set_wheel_motors(0.0, 0.0)
			done = True

		# display the image with any overlays

		# puck overlay
		if puck.found:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv.circle(cvImage, (int(puck.x), int(puck.y)), int(puck.radius),
				(0, 255, 255), 2)
			cv.circle(cvImage, puck.center, 5, (0, 0, 255), -1)

		if displayImage:
			cv.imshow('Vector', cvImage)

		# waitKey performs the display update
		# and checks to see if we hit the 'q' key
		c = cv.waitKey(MainLoopDelay)
		if (chr(c & 0xff) == 'q'):
			done = True

	allDone(robot)


if __name__ == "__main__":
    main()
