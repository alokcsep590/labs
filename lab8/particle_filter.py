import math
import numpy as np
import random

from grid import *
from particle import Particle
from utils import *
from setting import *

# translational difference allow
Diff_trans = 1.5
# orientation difference allow in degree
Diff_rot = 15

def probability(mean, sigma, val):
    return (1/math.sqrt(2*math.pi*math.pow(sigma,2)))*math.exp(-math.pow(val-mean, 2)/(2*math.pow(sigma,2)))
    
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
        
    measured_particles = []
    
    weight_particles = []
    
    insideGrid_particles = []
    
    for particle in particles[:]:
        if grid.is_in(particle.x, particle.y):            
            weight_particles.append(1)
        else :
            weight_particles.append(0)
    
    #print("particles")
    #print(particles)
        
    # avoid round-off to zero

    weight_particles = [float(i) + 1.e-300 for i in weight_particles]
    
    norm_weight_particles = [float(i)/sum(weight_particles) for i in weight_particles]
    
    indexes = systematic_resample(norm_weight_particles)
        
    for index in indexes[:]:
        insideGrid_particles.append(particles[index])
    
    weight_particles = [1.0] * len(weight_particles)
    
    #calculate mean, sigma for grid markers
    mean_x = 0
    mean_y = 0
    mean_h = 0
    sigma_x = 0
    sigma_y = 0
    sigma_h = 0
    for gmarker in grid.markers[:]:
        m_x, m_y, m_h = parse_marker_info(gmarker[0], gmarker[1], gmarker[2])
        mean_x += m_x
        mean_y += m_y
        mean_h += m_h
    mean_x = mean_x/len(grid.markers)
    mean_y = mean_y/len(grid.markers)
    mean_h = mean_h/len(grid.markers)
    
    for gmarker in grid.markers[:]:
        m_x, m_y, m_h = parse_marker_info(gmarker[0], gmarker[1], gmarker[2])
        sigma_x += math.pow(m_x - mean_x, 2)
        sigma_y += math.pow(m_y - mean_x, 2)
        sigma_h += math.pow(m_h - mean_x, 2)
    sigma_x = math.sqrt(sigma_x/len(grid.markers))
    sigma_y = math.sqrt(sigma_y/len(grid.markers))
    sigma_h = math.sqrt(sigma_h/len(grid.markers))
    
    for idx, insideGrid_particle in enumerate(insideGrid_particles):
        #num_markers_observed = 0
        for rmarker in measured_marker_list[:]:
            #print("rmarker")
            #print(rmarker)
            alpha = (math.pi*insideGrid_particle.h)/180
            marker_x = insideGrid_particle.x + (rmarker[0]*math.cos(alpha) - rmarker[1]*math.sin(alpha)) 
            marker_y = insideGrid_particle.y + (rmarker[0]*math.sin(alpha) + rmarker[1]*math.cos(alpha))
            marker_h = insideGrid_particle.h + rmarker[2]
            #print("marker")
            #print(marker_x, marker_y, marker_h)
            weight_particles[idx] *= probability(mean_x, sigma_x, marker_x)*probability(mean_y, sigma_y, marker_y)*probability(mean_h, sigma_h, marker_h)


        """
            for gmarker in grid.markers[:]:
                m_x, m_y, m_h = parse_marker_info(gmarker[0], gmarker[1], gmarker[2])
                if grid_distance(marker_x, marker_y, m_x, m_y) < Diff_trans \
                        and math.fabs(marker_h - m_h) < Diff_rot:
                    num_markers_observed += 1
                    #print("num_markers_observed = ")
                    #print(num_markers_observed)
                    break
        if len(measured_marker_list) == num_markers_observed:
            print("insideGrid_particle=")
            print(insideGrid_particle)
            weight_particles[idx] = weight_particles[idx]+1.0
        """
            
    weight_particles = [float(i) + 1.e-300 for i in weight_particles]
    norm_weight_particles = [float(i)/sum(weight_particles) for i in weight_particles]
    
    indexes = systematic_resample(norm_weight_particles)
        
    for index in indexes[:]:
        measured_particles.append(insideGrid_particles[index])
        
    #print("particles after resampling")
    #print(measured_particles)
        
    return measured_particles   


