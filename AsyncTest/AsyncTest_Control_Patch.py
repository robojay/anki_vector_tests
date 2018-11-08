#
# AsyncTest with request and release control
#

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
import time
from anki_vector.events import Events

def main():
	# grab serial number of robot if required
	args = anki_vector.util.parse_command_args()

	# use AsyncRobot so that behaviors don't block
	robot = anki_vector.AsyncRobot(args.serial, request_control=False, default_logging=False)
	robot.connect()
	time.sleep(1)

	robot.conn.request_control()
	controlRequest = robot.conn.run_coroutine(robot.conn.control_granted_event.wait())
	while not(controlRequest.done()):
		pass

	chargerAction = robot.behavior.get_
	while not(liftAction.done()):
		pass

	robot.release_control()

	#clean up
	robot.disconnect()


# AsyncRobot and Event Handling example
# along with a simple state machine
# by RoboJay.us

import anki_vector
from anki_vector.events import Events
import time

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
	print()
	print('I see something!')
	print(info)
	print()

# Main processing code
def main():
	# setup and connect to the robot
	# note the use of 'AsyncRobot' instead of 'Robot'
	# this allows code to continue while behaviors are running

	# grab serial number of robot if required
	args = anki_vector.util.parse_command_args()

	# use AsyncRobot so that behaviors don't block
	robot = anki_vector.AsyncRobot(args.serial, request_control=False, default_logging=False)
	robot.connect()
	time.sleep(1)

	isOnCharger = (robot.status & IS_ON_CHARGER) == IS_ON_CHARGER
	if not(isOnCharger):
		print('Vector needs to start on the charger')
		print('Put him there and re-run the program...')
		robot.disconnect()
		exit()

	# subscribe to observed object events
	robot.events.subscribe(on_robot_observed_object, Events.robot_observed_object)

	# request control and wait until we have it
	print('Requesting control')
	robot.conn.request_control()
	controlRequest = robot.conn.run_coroutine(robot.conn.control_granted_event.wait())
	while not(controlRequest.done()):
		print('...waiting for control')
		time.sleep(1)

	print('Running away')
	action = robot.behavior.drive_off_charger()

	robotState = 'Leaving charger'

	# watch how the robot moves around while the code here is counting
	for i in range(0,30):
		# print a counter
		print(i)

		# check our motion status
		isMoving = (robot.status & IS_MOVING) == IS_MOVING
		isOnCharger = (robot.status & IS_ON_CHARGER) == IS_ON_CHARGER

		# simple state machine
		# this runs every time the loop executes

		if robotState == 'Leaving charger':
			if action.done():
				robotState = 'Scared'
			else:
				print('...leaving')
				robotState = 'Leaving charger'

		elif robotState == 'Scared':
			print('Nah, going home')
			action = robot.behavior.drive_on_charger()
			robotState = 'Run away'

		elif robotState == 'Run away':
			if not(action.done()) and not(isOnCharger):
				print('...going home')
				robotState = 'Run away'
			else:
				robotState = 'Home sweet home'

		elif robotState == 'Home sweet home':
			print('Time for a nap.')
			robotState = 'Nap time'

		elif robotState == 'Nap time':
			print('snore...')
			robotState = 'Nap time'

		else:
			print('Uh oh, this is a bad state.')			
			print('Human assistance is needed.')			

		# the loop executes every second
		time.sleep(1)

	#clean up
	print('Releasing control')
	robot.release_control()

	robot.disconnect()


# runs our main code
if __name__ == "__main__":
    main()




# runs our main code
if __name__ == "__main__":
    main()

