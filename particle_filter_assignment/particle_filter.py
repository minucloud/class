#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

# World and visualization imports
from simulator import World, Robot, Visualizer

# Particle filter imports
from core.particle_filters import ParticleFilterSIR
from core.resampling import ResamplingAlgorithms

class ParticleFilterDemo:
    
    def __init__(self):
        self.world = World(10.0, 10.0, [[2.0, 2.0], [2.0, 8.0], [9.0, 2.0], [8, 9]])
        self.robot_setpoint_motion_forward = 0.25
        self.robot_setpoint_motion_turn = 0.02
        self.true_robot_motion_forward_std = 0.005
        self.true_robot_motion_turn_std = 0.002
        self.true_robot_meas_noise_distance_std = 0.2
        self.true_robot_meas_noise_angle_std = 0.05
        self.number_of_particles = 1000
        self.pf_state_limits = [0, self.world.x_max, 0, self.world.y_max]
        self.motion_model_forward_std = 0.1
        self.motion_model_turn_std = 0.20
        self.process_noise = [self.motion_model_forward_std, self.motion_model_turn_std]
        self.meas_model_distance_std = 0.4
        self.meas_model_angle_std = 0.3
        self.measurement_noise = [self.meas_model_distance_std, self.meas_model_angle_std]
        self.algorithm = ResamplingAlgorithms.MULTINOMIAL
        self.particle_filter_sir = ParticleFilterSIR(
            number_of_particles=self.number_of_particles,
            limits=self.pf_state_limits,
            process_noise=self.process_noise,
            measurement_noise=self.measurement_noise,
            resampling_algorithm=self.algorithm)
        self.particle_filter_sir.initialize_particles_uniform()
        self.visualizer = Visualizer(False)
        self.visualizer.update_robot_radius(0.2)
        self.visualizer.update_landmark_size(7)
        
    def run(self):
        n_time_steps = 30
        robot = Robot(x=self.world.x_max * 0.75,
                      y=self.world.y_max / 5.0,
                      theta=3.14 / 2.0,
                      std_forward=self.true_robot_motion_forward_std,
                      std_turn=self.true_robot_motion_turn_std,
                      std_meas_distance=self.true_robot_meas_noise_distance_std,
                      std_meas_angle=self.true_robot_meas_noise_angle_std)
        for i in range(n_time_steps):
            robot.move(desired_distance=self.robot_setpoint_motion_forward,
                       desired_rotation=self.robot_setpoint_motion_turn,
                       world=self.world)
            measurements = robot.measure(self.world)
            self.particle_filter_sir.update(robot_forward_motion=self.robot_setpoint_motion_forward,
                                            robot_angular_motion=self.robot_setpoint_motion_turn,
                                            measurements=measurements,
                                            landmarks=self.world.landmarks)
            self.visualizer.draw_world(self.world, robot, self.particle_filter_sir.particles, hold_on=False, particle_color='g')
            plt.pause(0.05)
    
if __name__ == '__main__':
    demo = ParticleFilterDemo()
    demo.run()
