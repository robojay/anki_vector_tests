import anki_vector
from anki_vector.events import Events
import cv2 as cv
import numpy as np
import time
import math

# Constants

Debug = False

# milliseconds per main loop execution
MainLoopDelay = 20

HeadTilt = anki_vector.robot.MIN_HEAD_ANGLE + anki_vector.util.degrees(5.0)
LiftHeight = 0.0

class Camera:
	FovH = anki_vector.util.degrees(90.0)
	FovV = anki_vector.util.degrees(50.0)
	PixelsH = 640.0
	PixelsV = 360.0
	FocalLengthH = (PixelsH / 2.0) / math.tan(FovH.radians / 2.0)

	DegressPerPixelH = FovH.degrees / PixelsH
	DegreesPerPixelV = FovV.degrees / PixelsV


class Ball:

	def __init__(self):

		# typical globals, but can be changed per instance if required
		# i.e. if more than one type and color of ball is used
		# these are based on the blue ball from a LEGO Mindstorms kit
		self.HsvMin = (105, 100, 10)
		self.HsvMax = (120, 200, 255)
		self.RadiusMax = 200.0
		self.RadiusMin = 15.0 # was 25
		self.RadiusScale = 38.0
		self.RadiusTolerance = 0.2
		self.AspectRatio = 1.0
		self.AspectRatioTolerance = 0.5
		self.AspectMin = (1 - self.AspectRatioTolerance) * self.AspectRatio
		self.AspectMax = (1 + self.AspectRatioTolerance) * self.AspectRatio
		self.TargetX = 310
		self.TargetY = 70

		self.found = False
		self.x = 0.0
		self.y = 0.0
		self.center = 0.0
		self.radius = 0.0
		self.deltaX = 0.0
		self.deltaY = 0.0
		self.imageId = 0
		self.distance = anki_vector.util.distance_mm(0.0)
		self.angle = anki_vector.util.degrees(0.0)
		self.mask = True
		self.debug = True
		self.testing = False

	# this will change depending on the size of the ball used, so shouldn't
	# really be hard coded here for a generic class...
	# hack... fix later... or override 
	def computeDistance(self, radius):
		# formula determined by measuring distance to ball versus
		# radius, entering into spreadsheet, calculating a regression formula
		distance = anki_vector.util.distance_mm(-174.1 * math.log(radius) + 852.5)
		return distance

	def computeAngle(self, x, camera):
		# measuring angle is based on camera FOV
		angle = anki_vector.util.radians(math.atan((x - (camera.PixelsH / 2.0)) / camera.FocalLengthH))
		return angle


	# find the ball in an image
	def findBall(self, imageId: int, image: np.array, maskImage: np.array, camera: Camera):

		# only do this if we have a new image
		if (imageId != self.imageId):
			self.imageId = imageId

			# Much information and some code was obtained from here:
			# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

			# blur, convert to HSV, look for the ball HSV values, do some filtering
			maskedImage = cv.bitwise_and(image, maskImage)
			blurImage = cv.GaussianBlur(maskedImage, (11, 11), 0)
			hsvImage = cv.cvtColor(blurImage, cv.COLOR_BGR2HSV)
			self.trackerImage = cv.inRange(hsvImage, self.HsvMin, self.HsvMax)
			self.trackerImage = cv.erode(self.trackerImage, None, iterations = 2)
			self.trackerImage = cv.dilate(self.trackerImage, None, iterations = 2)

			# find contours
			im2, contours, hierarchy = cv.findContours(self.trackerImage.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

			# We're given a a bunch of contours that might enclose the ball
			# To help decide if a contour is a ball:
			#   - pick the largest contour
			#   - find the radius of the enclosing circle (scale based on distance)
			#   - compute the aspect ratio
			# If the radius and aspect ratio are within the tolerances of a ball,
			# it most likely is a ball.  However, some features on the rink can
			# still come close to looking like a ball... more work could be done.

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
		 
				deltaX = self.TargetX - int(x) # positive is to the left
				deltaY = self.TargetY - int(y) # positive is away

				# compute angle and distance based on radius
				# formula determined by measuring distance to ball versus
				# radius, entering into spreadsheet, calculating a regression formula
				# measuring angle is based on camera FOV
				#
				# this will change depending on the size of the ball used, so shouldn't
				# really be hard coded here for a generic class...
				# hack... fix later...
				distance = self.computeDistance(radius)
				angle = self.computeAngle(x, camera)

				if (self.debug):
					print(f'[findBall] ({x:.0f},{y:.0f}) ({deltaX:.0f},{deltaY:.0f}) R = {radius:.2f}')
					print(f'[findBall] width = {bW:.0f} height = {bH:.0f} aspect = {aspectRatio:.2f}')
					print(f'[findBall] angle = {angle.degrees:.2f} distance = {distance.distance_mm:.2f}')

				# perform the actual checks
				if ( (self.RadiusMin < radius < self.RadiusMax) and (self.AspectMin < aspectRatio < self.AspectMax) or
						self.testing ):
					if (self.debug):
						print('[findBall] Got it')
					self.found = True
					self.x = x
					self.y = y
					self.center = center
					self.radius = radius
					self.deltaX = deltaX
					self.deltaY = deltaY
					self.distance = distance
					self.angle = angle

				else:
					self.found = False




# left of center is negative
def goToObject(robot: anki_vector.Robot, distance: anki_vector.util.Distance, angle: anki_vector.util.Angle):
	global camera

	pDistance = 1.0
	pAngle = 1.0
	leftSpeed = pDistance * distance.distance_mm + pAngle * angle.degrees
	rightSpeed = pDistance * distance.distance_mm - pAngle * angle.degrees
	#print(f'[goToPixel] {leftSpeed:.1f}:{rightSpeed:.1f}') 
	robot.motors.set_wheel_motors(leftSpeed, rightSpeed)

def lookAround(robot: anki_vector.Robot, scanAngleIndex: int):
	ScanAngles = [0, -30, 30, -60, 60, -90, 90, -120, 120, -150, 150, 180]

	if (actionsDone()):
		robot.motors.set_wheel_motors(0, 0)
		scanAngleIndex += 1
		if (scanAngleIndex == len(ScanAngles)):
			scanAngleIndex = 0
		action(robot.behavior.turn_in_place(angle=anki_vector.util.degrees(ScanAngles[scanAngleIndex]), 
			speed=anki_vector.util.degrees(60.0),
			is_absolute=True))
		
	return scanAngleIndex


def allDone(robot: anki_vector.Robot):
	print('[allDone] Cleaning up')
	robot.motors.set_wheel_motors(0, 0)
	cv.destroyAllWindows()
	robot.disconnect()
	exit()

def actionsDone():
	global actionList
	done = True
	for i in actionList:
		if not(i.done()):
			done = False
	return done

def action(b):
	global actionList
	actionList.append(b)


def main():	
	global actionList

	actionList = []

	ball = Ball()
	camera = Camera()

	cvImageId = 0
	scanAngleIndex = 0

	# open the video window
	cv.namedWindow('Vector', cv.WINDOW_NORMAL)
	if ball.testing:
		cv.namedWindow('Tracker', cv.WINDOW_NORMAL)

	# read in the mask for Vector's lift (all the way down)
	vectorMaskImage = cv.imread('Vector_Mask.png')

	args = anki_vector.util.parse_command_args()
	# use AsyncRobot so that behaviors don't block
	robot = anki_vector.AsyncRobot(args.serial, enable_camera_feed=True, default_logging=Debug)
	robot.connect()
	time.sleep(1)

	done = False
	displayImage = False

	action(robot.behavior.set_head_angle(HeadTilt))
	action(robot.behavior.set_lift_height(LiftHeight))
	while not(actionsDone()):
		pass

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

			# locate the ball (if we can see it)
			ball.findBall(imageId = cvImageId, image = cvImage, maskImage = vectorMaskImage, camera = camera)

			# ball overlay
			if ball.found:
				goToObject(robot, ball.distance, ball.angle)

				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv.circle(cvImage, (int(ball.x), int(ball.y)), int(ball.radius),
					(0, 255, 255), 2)
				cv.circle(cvImage, ball.center, 5, (0, 0, 255), -1)

			else:
				#scanAngleIndex = lookAround(robot, scanAngleIndex)
				robot.motors.set_wheel_motors(0, 0)


			# display the image with any overlays
			cv.imshow('Vector', cvImage)
			if ball.testing:
				cv.imshow('Tracker', ball.trackerImage)

		# waitKey performs the display update
		# and checks to see if we hit the 'q' key
		c = cv.waitKey(MainLoopDelay)
		if (chr(c & 0xff) == 'q'):
			done = True

	allDone(robot)


if __name__ == "__main__":
	main()
