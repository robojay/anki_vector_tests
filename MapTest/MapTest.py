#!/usr/bin/env python3

import anki_vector
from anki_vector.events import Events
from anki_vector.util import degrees
import time
import functools


def on_nav_map_update(robot, event_type, event):
	print('Map Update')
	map = robot.nav_map.latest_nav_map
	print('***')
	print(map.center)
	print(map.size)
	print('===')
	recurse_map(map.root_node)

def recurse_map(node):
	if node.children == None:
		print('---')
		print(node.center)
		print(node.size)
		print(node.content)
	else:
		for child in node.children:
			recurse_map(child)
	return

def main():
	args = anki_vector.util.parse_command_args()
	robot = anki_vector.Robot(serial = args.serial, 
									default_logging = True,
									enable_nav_map_feed=True)

	robot.connect()
	pretty_name = args.serial
	if pretty_name == None:
		pretty_name = ''
	#robot.say_text("Vector %s Ready" % pretty_name)

	# subscribe to nav map update events
	on_nav_map_update_mod = functools.partial(on_nav_map_update, robot)
	robot.events.subscribe(on_nav_map_update_mod, Events.nav_map_update)

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


	# robot.events.unsubscribe(on_nav_map_update, Events.nav_map_update)
	#robot.say_text("All Done")
	robot.disconnect()

if __name__ == "__main__":
	main()
