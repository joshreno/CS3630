from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np

# Authors: Joshua Reno, Giuseppe Pantalone

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    dx, dy, dh = odom
    particlesList = []
    for particle in particles:
        x, y = rotate_point(dx, dy, particle.h)
        particle.x += add_gaussian_noise(x, ODOM_TRANS_SIGMA)
        particle.y += add_gaussian_noise(y, ODOM_TRANS_SIGMA)
        particle.h += add_gaussian_noise(dh, ODOM_HEAD_SIGMA)
        particlesList.append(particle)
    return particlesList

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    measuredMarkerListLength, num, weights = len(measured_marker_list), 0, []
    for particle in particles:
        if measuredMarkerListLength:
            robotMarkers = particle.read_markers(grid = grid)
            markersLength = len(robotMarkers)
            if not markersLength: weights.append(0)
            elif not grid.is_free(x = particle.x, y = particle.y):
                weights.append(0)
                num += 1
            else:
                lst, confidence = [], 1
                for measured_marker in measured_marker_list:
                    if markersLength:
                        worst = min(robotMarkers, key = lambda m: grid_distance(x1 = measured_marker[0], y1 = measured_marker[1], x2 = m[0], y2 = m[1]))
                        robotMarkers.remove(worst)
                        markersLength = len(robotMarkers)
                        lst.append((measured_marker, worst))
                for marker, worst in lst: confidence *= math.exp(-1 * (math.pow(grid_distance(marker[0], 
                    marker[1], worst[0], worst[1]), 2) / (2 * math.pow(MARKER_TRANS_SIGMA, 2)) 
                + math.pow(diff_heading_deg(marker[2], worst[2]), 2) / (2 * math.pow(MARKER_ROT_SIGMA, 2))))
                weights.append(confidence)
    particleLength, weightsSum = len(particles), sum(weights)
    if measuredMarkerListLength < 1 or weightsSum == 0: weights = particleLength * [(1 / float(particleLength))]
    else: weights = [weight / weightsSum for weight in weights]
    beliefList = [Particle(x = particle.x, y = particle.y, heading = particle.h) for particle in np.random.choice(particles, size = (particleLength - min(particleLength, 50 + num)), p = weights).tolist()]
    for x, y in [grid.random_free_place() for i in range(min(particleLength, 50 + num))]: beliefList.append(Particle(x = x, y = y))
    return beliefList
    

