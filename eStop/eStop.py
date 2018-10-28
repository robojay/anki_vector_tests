import anki_vector
import time

with anki_vector.Robot() as robot:
	time.sleep(1)
	robot.motors.set_wheel_motors(0.0, 0.0)
	time.sleep(1.0)
