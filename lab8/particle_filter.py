import math
import numpy as np
import copy
import numpy.random as npr

from grid import *
from particle import Particle
from utils import *
from setting import *
from scipy.stats import norm

def multinomial_resample(weights):
    cumulative_sum = np.cumsum(weights)

    cumulative_sum[-1] = 1.  # avoid round-off errors: ensures sum is exactly one

    return np.searchsorted(cumulative_sum, npr.random(len(weights)))
    
def systematic_resample(weights):
    N = len(weights)

    # make N subdivisions, choose positions 
    # with a consistent random offset
    positions = (np.arange(N) + random.random()) / N

    indexes = np.zeros(N, 'i')
    cumulative_sum = np.cumsum(weights)
    i, j = 0, 0
    while i < N:
        if positions[i] < cumulative_sum[j]:
            indexes[i] = j
            i += 1
        else:
            j += 1
    return indexes

def stratified_resample(weights):
    N = len(weights)

    # make N subdivisions, and chose a random position within each one

    positions = (npr.random(N) + range(N)) / N



    indexes = np.zeros(N, 'i')

    cumulative_sum = np.cumsum(weights)

    i, j = 0, 0

    while i < N:

        if positions[i] < cumulative_sum[j]:

            indexes[i] = j

            i += 1

        else:

            j += 1

    return indexes

    
def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []
                   
    for particle in particles[:]:
    
        # Add odometry noise to each particle (this noise is needed since all particles won't move same way)
        odom_with_noise = add_odometry_noise(odom, heading_sigma=ODOM_HEAD_SIGMA, trans_sigma=ODOM_TRANS_SIGMA)

        alpha = (math.pi*particle.h)/180
        particle.x = particle.x + (odom_with_noise[0]*math.cos(alpha) - odom_with_noise[1]*math.sin(alpha)) 
        particle.y = particle.y + (odom_with_noise[0]*math.sin(alpha) + odom_with_noise[1]*math.cos(alpha))
        particle.h = particle.h + odom_with_noise[2]
        
        motion_particles.append(particle)
            
    return motion_particles

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
    

    weight_particles = []
    for particle in particles:
        if  grid.is_in(particle.x, particle.y):
            rmarker_weight = 1
        else:
            rmarker_weight = 0
            
        particle_marker_list = particle.read_markers(grid)
        for rmarker in measured_marker_list[:]:
            pmarker_weight = 0
            for pmarker in particle_marker_list[:]: 
                pmarker_weight += norm.pdf(rmarker[0],pmarker[0],MARKER_TRANS_SIGMA)*norm.pdf(rmarker[1],pmarker[1],MARKER_TRANS_SIGMA)*norm.pdf(rmarker[2],pmarker[2],MARKER_ROT_SIGMA)            
            rmarker_weight *= pmarker_weight
        weight_particles.append(rmarker_weight)
    
    norm_weight_particles = [float(i)/sum(weight_particles) for i in weight_particles]

    resample_count = int(0.98*len(particles))
    measured_particles = np.random.choice(particles, resample_count, p = norm_weight_particles)
    
    new_measured_particles = []
    for m_particle in measured_particles:
        new_measured_particles.append(copy.deepcopy(m_particle))
        
        
    new_particles = Particle.create_random(len(particles)-resample_count, grid)
    for new_particle in new_particles:
        new_measured_particles.append(copy.deepcopy(new_particle))
    
    return new_measured_particles