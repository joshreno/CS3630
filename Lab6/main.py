import cozmo
# from Lab1_Soln import ImageClassifier
from cozmo.util import degrees, distance_mm, speed_mmps

import sklearn
from sklearn.externals import joblib
import numpy

from go_to_goal import *
from rrt import *
from cmap import *

cube_list = []
markers = None

def run(robot: cozmo.robot.Robot):
	robot.camera.image_stream_enabled = True
	robot.camera.color_image_enabled = False
	robot.camera.enable_auto_exposure()

def localize():
	cozmo_thread = CozmoThread()
	cozmo_thread.start()
	gui.show_particles(pf.particles)
	gui.show_mean(0, 0, 0)
	gui.start()
	return x(), y(), z(), markers()

def getCubes(a, b, c):
	rrt.a(a)
	rrt.b(b)
	rrt.c(c)
	cube = None
	marker = None
	for i in len(3):
		cube = robot.world.wait_for_observed_light_cube(timeout=30)
		robot.pickup_object(cube, num_retries=5).wait_for_completed()
		pose = robot.pose
		obj = robot.world.visible_objects
		if obj.object_id in cube_list: 
			i = i - 1
			# turn in place
			robot.turn_in_place(angle = 20)
			continue
		else:
			cube_list.append(obj.object_id)
		x = obj.pose.position.x
		y = obj.pose.position.y
		if x < 100 and y < 100: marker = 'drone'
		elif x < 100 and y > 500: marker = 'inspection'
		elif x > 500 and y > 500: marker = 'place'
		else: marker = 'plane'
		cmap = CozMap("maps/emptygrid.json", node_generator)
		cmap.add_goal(markers[marker])
		robot_thread = RobotThread()
		robot_thread.start()

def cozmoMethod(robot: cozmo.robot.Robot):
	run(robot)
	x, y, z, markers = localize()
	getCubes(x, y, z)

cozmo.run_program(cozmoMethod)