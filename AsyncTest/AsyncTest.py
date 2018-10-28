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

#
# This example would also be great if there was a Behavior Complete event...
#

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
	robot = anki_vector.AsyncRobot(default_logging=False)
	robot.connect()
	time.sleep(1)

	# subscribe to observed object events
	robot.events.subscribe(on_robot_observed_object, Events.robot_observed_object)

	print('Running away')
	robot.behavior.drive_off_charger()

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
			if isMoving:
				robotState = 'Move along'
			else:
				print('...thinking about it')
				robotState = 'Leaving charger'
		elif robotState == 'Move along':
			if isMoving:
				print('...move along')
				robotState = 'Move along'
			else:
				robotState = 'Scared'
		elif robotState == 'Scared':
			print('Nah, going home')
			robot.behavior.drive_on_charger()
			robotState = 'I wanna go home'
		elif robotState == 'I wanna go home':
			if isMoving:
				robotState = 'Run away'
			else:
				print('...thinking about it')
				robotState = 'I wanna go home'
		elif robotState == 'Run away':
			if not(isOnCharger):
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
	robot.disconnect()


# runs our main code
if __name__ == "__main__":
    main()

