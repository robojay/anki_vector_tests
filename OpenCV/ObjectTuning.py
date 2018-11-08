#
# This uses the SDK patch provided by @wvenable on the Vector SDK forum.
#
# Post describing the patch can be found here:
# https://forums.anki.com/t/interact-with-vector-without-stopping-built-in-behaviors/21475
#
# The patch itself can be found here: 
# https://pastebin.com/30PdPnrB
# 
import sys

# If your patched SDK is not in the Python Sites
# point to it here, if it is in Sites, comment these two lines out 
sdkPath = '/Users/jay/Development/Vector/PatchedSDK'
sys.path.insert(1,sdkPath)

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

# Red Sportcraft "Powerpuck" round air hockey puck
class RedPuck:
	debug = True
	testing = False

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
	distance = anki_vector.util.distance_mm(0.0)
	angle = anki_vector.util.degrees(0.0)


# Blue ball from Lego Mindstorms
class BlueBall:
	debug = False
	testing = False

	HsvMin = (105, 100, 10)
	HsvMax = (120, 200, 255)
	RadiusMax = 200.0
	RadiusMin = 25.0
	RadiusScale = 38.0
	RadiusTolerance = 0.2
	AspectRatio = 1.0
	AspectRatioTolerance = 0.5
	AspectMin = (1 - AspectRatioTolerance) * AspectRatio
	AspectMax = (1 + AspectRatioTolerance) * AspectRatio
	TargetX = 310
	TargetY = 70

	found = False
	x = 0.0
	y = 0.0
	center = 0.0
	radius = 0.0
	deltaX = 0.0
	deltaY = 0.0
	imageId = 0
	distance = anki_vector.util.distance_mm(0.0)
	angle = anki_vector.util.degrees(0.0)


class Camera:
	FovH = anki_vector.util.degrees(90.0)
	FovV = anki_vector.util.degrees(50.0)
	PixelsH = 640.0
	PixelsV = 360.0
	FocalLengthH = (PixelsH / 2.0) / math.tan(FovH.radians / 2.0)

	DegressPerPixelH = FovH.degrees / PixelsH
	DegreesPerPixelV = FovV.degrees / PixelsV

HeadTilt = anki_vector.robot.MIN_HEAD_ANGLE + anki_vector.util.degrees(5.0)
LiftHeight = 0.0

puck = RedPuck()
ball = BlueBall()
camera = Camera()

# find the ball in an image
def findBall():
	global cvImage
	global trackerImage
	global cvImageId
	global ball
	global vectorMaskImage
	global camera

	# only do this if we have a new image
	if (cvImageId != ball.imageId):
		ball.imageId = cvImageId

		# Much information and some code was obtained from here:
		# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

		# blur, convert to HSV, look for the ball HSV values, do some filtering
		maskedImage = cv.bitwise_and(cvImage, vectorMaskImage)
		blurImage = cv.GaussianBlur(maskedImage, (11, 11), 0)
		hsvImage = cv.cvtColor(blurImage, cv.COLOR_BGR2HSV)
		trackerImage = cv.inRange(hsvImage, ball.HsvMin, ball.HsvMax)
		#trackerImage = cv.bitwise_and(trackerImage, vectorMaskImage)
		trackerImage = cv.erode(trackerImage, None, iterations = 2)
		trackerImage = cv.dilate(trackerImage, None, iterations = 2)

		# find contours
		im2, contours, hierarchy = cv.findContours(trackerImage.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

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
	 
			deltaX = ball.TargetX - int(x) # positive is to the left
			deltaY = ball.TargetY - int(y) # positive is away

			# compute angle and distance based on radius
			# formula determined by measuring distance to ball versus
			# radius, entering into spreadsheet, calculating a regression formula
			# measuring angle is based on camera FOV
			distance = anki_vector.util.distance_mm(-174.1 * math.log(radius) + 852.5)
			angle = anki_vector.util.radians(math.atan((x - (camera.PixelsH / 2.0)) / camera.FocalLengthH))

			# --this does not work for a ball--
			# compute the radius we want to check against based on distance in the Y
			# direction of the video frame
			#radiusCheck = ball.RadiusMax - ball.RadiusScale * deltaY 

			# uncomment for debugging ball issues
			if (ball.debug):
				print(f'[findBall] ({x:.0f},{y:.0f}) ({deltaX:.0f},{deltaY:.0f}) R = {radius:.2f}')
				print(f'[findBall] width = {bW:.0f} height = {bH:.0f} aspect = {aspectRatio:.2f}')
				print(f'[findBall] angle = {angle.degrees:.2f} distance = {distance.distance_mm:.2f}')
				#print(f'[findBall] radiusCheck = {radiusCheck:.2f}')

			# only proceed if the radius meets a certain size
			#radiusMin = (1 - ball.RadiusTolerance) * radiusCheck
			#radiusMax = (1 + ball.RadiusTolerance) * radiusCheck

			#print(f'[findBall] radiusMin = {radiusMin:.2f} radiusMax = {radiusMax:.2f}')

			# perform the actual checks
			if ( (ball.RadiusMin < radius < ball.RadiusMax) and (ball.AspectMin < aspectRatio < ball.AspectMax) ):
				if (ball.debug):
					print('[findBall] Got it')
				ball.found = True
				ball.x = x
				ball.y = y
				ball.center = center
				ball.radius = radius
				ball.deltaX = deltaX
				ball.deltaY = deltaY
				ball.distance = distance
				ball.angle = angle

			else:
				ball.found = False

			# testing, forces ball to be found
			if (ball.testing):
				ball.found = True
				ball.x = x
				ball.y = y
				ball.center = center
				ball.radius = radius
				ball.deltaX = deltaX
				ball.deltaY = deltaY
				ball.distance = distance


# find the puck in an image
def findPuck():
	global cvImage
	global trackerImage
	global cvImageId
	global puck
	global vectorMaskImage

	# only do this if we have a new image
	if (cvImageId != puck.imageId):
		puck.imageId = cvImageId

		# Much information and some code was obtained from here:
		# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

		# blur, convert to HSV, look for the puck HSV values, do some filtering
		maskedImage = cv.bitwise_and(cvImage, vectorMaskImage)
		blurImage = cv.GaussianBlur(maskedImage, (11, 11), 0)
		hsvImage = cv.cvtColor(blurImage, cv.COLOR_BGR2HSV)
		trackerImage = cv.inRange(hsvImage, puck.HsvMin, puck.HsvMax)
		#trackerImage = cv.bitwise_and(trackerImage, vectorMaskImage)
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

			# compute the radius we want to check against based on distance in the Y
			# direction of the video frame
			radiusCheck = puck.RadiusMax - puck.RadiusScale * deltaY 

			# uncomment for debugging puck issues
			if (puck.debug):
				print(f'[findPuck] ({x:.0f},{y:.0f}) ({deltaX:.0f},{deltaY:.0f}) R = {radius:.2f}')
				print(f'[findPuck] width = {bW:.0f} height = {bH:.0f} aspect = {aspectRatio:.2f}')
				print(f'[findPuck] radiusCheck = {radiusCheck:.2f}')

			# only proceed if the radius meets a certain size
			radiusMin = (1 - puck.RadiusTolerance) * radiusCheck
			radiusMax = (1 + puck.RadiusTolerance) * radiusCheck

			print(f'[findPuck] radiusMin = {radiusMin:.2f} radiusMax = {radiusMax:.2f}')

			# perform the actual checks
			if ( (radiusMin < radius < radiusMax) and (puck.AspectMin < aspectRatio < puck.AspectMax) ):
				if (puck.debug):
					print('[findPuck] Got it')
				puck.found = True
				puck.x = x
				puck.y = y
				puck.center = center
				puck.radius = radius
				puck.deltaX = deltaX
				puck.deltaY = deltaY

			else:
				puck.found = False


			# testing, forces puck to be found
			if (puck.testing):
				puck.found = True
				puck.x = x
				puck.y = y
				puck.center = center
				puck.radius = radius
				puck.deltaX = deltaX
				puck.deltaY = deltaY

def allDone(robot: anki_vector.Robot):
	print('[allDone] Cleaning up')
	robot.disconnect()
	cv.destroyAllWindows()
	exit()


def main():	
	# this will be the OpenCV version of robot.camera.latest_image
	global cvImage
	global cvImageId
	global trackerImage
	global puck
	global ball
	global camera
	global vectorMaskImage

	cvImageId = 0

	# open the video window
	cv.namedWindow('Vector', cv.WINDOW_NORMAL)
	cv.namedWindow('Tracker', cv.WINDOW_NORMAL)

	# read in the mask for Vector's lift (all the way down)
	vectorMaskImage = cv.imread('Vector_Mask.png')

	# use AsyncRobot so that behaviors don't block
	robot = anki_vector.AsyncRobot(enable_camera_feed=True, default_logging=Debug)
	robot.connect()
	time.sleep(1)

	done = False
	displayImage = False

	headAction = robot.behavior.set_head_angle(HeadTilt)
	while not(headAction.done()):
		pass
	liftAction = robot.behavior.set_lift_height(LiftHeight)
	while not(liftAction.done()):
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

		if (False):
			# locate the puck (if we can see it)
			findPuck()

			# display the image with any overlays

			# puck overlay
			if puck.found:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv.circle(cvImage, (int(puck.x), int(puck.y)), int(puck.radius),
					(0, 255, 255), 2)
				cv.circle(cvImage, puck.center, 5, (0, 0, 255), -1)

		if (True):
			# locate the ball (if we can see it)
			findBall()

			# display the image with any overlays

			# ball overlay
			if ball.found:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv.circle(cvImage, (int(ball.x), int(ball.y)), int(ball.radius),
					(0, 255, 255), 2)
				cv.circle(cvImage, ball.center, 5, (0, 0, 255), -1)

		# cvImage = cv.bitwise_and(cvImage, vectorMaskImage)

		if displayImage:
			cv.imshow('Vector', cvImage)
			cv.imshow('Tracker', trackerImage)

		# waitKey performs the display update
		# and checks to see if we hit the 'q' key
		c = cv.waitKey(MainLoopDelay)
		if (chr(c & 0xff) == 'q'):
			done = True

	allDone(robot)


if __name__ == "__main__":
    main()
