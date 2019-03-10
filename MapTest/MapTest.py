#!/usr/bin/env python3

import anki_vector
from anki_vector.events import Events
from anki_vector.util import degrees
import time

def on_nav_map_update(event, info):
	print('Map Update')

def main():
	args = anki_vector.util.parse_command_args()
	robot = anki_vector.Robot(serial = args.serial, 
									default_logging = True,
									enable_nav_map_feed=True)

	robot.connect()
	pretty_name = args.serial
	if pretty_name == None:
		pretty_name = ''
	robot.say_text("Vector %s Ready" % pretty_name)

	# subscribe to nav map update events
	robot.events.subscribe(on_nav_map_update, Events.nav_map_update)

	if not(robot.status.is_on_charger):
		print('Vector needs to start on the charger')
		print('Put him there and re-run the program...')
		robot.say_text("Uh Oh")
		robot.disconnect()
		exit()

	# make Vector wander a bit
	robot.behavior.drive_off_charger()

	robot.behavior.turn_in_place(angle = degrees(360), speed = degrees(45))

	robot.behavior.drive_on_charger()

	robot.say_text("All Done")
	robot.disconnect()

if __name__ == "__main__":
	main()
