import anki_vector
import time

args = anki_vector.util.parse_command_args()
with anki_vector.Robot(args.serial) as robot:
	time.sleep(1)
	robot.motors.set_wheel_motors(0.0, 0.0)
	time.sleep(1.0)
