import cozmo
from Lab1_Soln import ImageClassifier
from cozmo.util import degrees, distance_mm, speed_mmps
import sklearn
from sklearn.externals import joblib
import numpy

def run(robot: cozmo.robot.Robot):
	robot.camera.image_stream_enabled = True
	robot.camera.color_image_enabled = False
	robot.camera.enable_auto_exposure()

def idle(robot: cozmo.robot.Robot):
	robot.set_head_angle(degrees(-5.0)).wait_for_completed()
	classifier = joblib.load('proj2classifier.pkl')
	images = []
	while len(images) < 3:
		if len(images) == 2: robot.turn_in_place(angle = cozmo.util.Angle(degrees = -5), speed = cozmo.util.Angle(degrees = 5)).wait_for_completed()
		else: robot.turn_in_place(angle = cozmo.util.Angle(degrees = 5), speed = cozmo.util.Angle(degrees = 5)).wait_for_completed()
		images.append(numpy.asarray(robot.world.latest_image.raw_image))
	labels = classifier.predict_labels(classifier.extract_image_features(images))
	temp = dict()
	for x in labels:
		if x in temp: temp[x] += 1
		else: temp[x] = 1
	maxNum = 0
	max_prediction = None
	for x in temp:
		tempX = temp[x]
		if tempX > maxNum:
			maxNum = tempX
			max_prediction = x
	if max_prediction == 'drone': drone(robot)
	elif max_prediction == 'order': order(robot)
	elif max_prediction == 'inspection': inspection(robot)
	else: idle(robot)

def drone(robot: cozmo.robot.Robot):
	# The robot should locate one of the cubes
	#(one will be placed in front of it within view),
	#pick up the cube, drive forward with the cube for 10cm,
	#put down the cube, and drive backward 10cm
	robot.say_text(str('drone')).wait_for_completed()
	robot.set_lift_height(height=0).wait_for_completed()
	cube = robot.world.wait_for_observed_light_cube(timeout=30)
	robot.pickup_object(cube, num_retries=5).wait_for_completed()
	robot.drive_straight(distance_mm(100), speed_mmps(50)).wait_for_completed()
	robot.place_object_on_ground_here(cube).wait_for_completed()
	robot.drive_straight(distance_mm(-100), speed_mmps(50)).wait_for_completed()
	idle(robot)

def order(robot: cozmo.robot.Robot):
	#Use the drive_wheels function to have the robot drive in a circle with an
	#approximate radius of 10cm
	robot.say_text(str('order')).wait_for_completed()
	robot.drive_wheels(l_wheel_speed=20, r_wheel_speed=40, duration=29)
	idle(robot)

def inspection(robot: cozmo.robot.Robot):
	#Have the robot drive in a square, where each side of the square is
	#approximately 20 cm. While driving, the robot must continuously raise
	#and lower the lift, but do so slowly (2-3 seconds to complete
	#lowering or raising the lift). Lower the lift at the end of the behavior
	robot.say_text(str('inspection')).wait_for_completed()
	for i in range(0, 4):
		if i % 2 == 0:
			robot.move_lift(0.25)
		else: 
			robot.move_lift(-0.25)
		robot.drive_straight(distance_mm(200), speed_mmps(50), in_parallel = True).wait_for_completed()
		robot.turn_in_place(degrees(90), in_parallel = True).wait_for_completed()
	idle(robot)

def cozmoMethod(robot: cozmo.robot.Robot):
	run(robot)
	idle(robot)

cozmo.run_program(cozmoMethod)
