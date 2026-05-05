from abc import abstractmethod
import copy
import numpy as np

from core.resampling import ResamplingAlgorithms
from core.resampling.resampler import Resampler

class ParticleFilterSIR():
    """
    Notes:
        * State is (x, y, heading), where x and y are in meters and heading in radians
        * State space assumed limited size in each dimension, world is cyclic (hence leaving at x_max means entering at
        x_min)
        * propagation and measurement models are largely hardcoded (except for standard deviations.
    """

    def __init__(self,
                 number_of_particles,
                 limits,
                 process_noise,
                 measurement_noise,
                 resampling_algorithm):
        """
        Initialize the SIR particle filter.

        :param number_of_particles: Number of particles.
        :param limits: List with maximum and minimum values for x and y dimension: [xmin, xmax, ymin, ymax].
        :param process_noise: Process noise parameters (standard deviations): [std_forward, std_angular].
        :param measurement_noise: Measurement noise parameters (standard deviations): [std_range, std_angle].
        :param resampling_algorithm: Algorithm that must be used for core.
        """


        if number_of_particles < 1:
            print("Warning: initializing particle filter with number of particles < 1: {}".format(number_of_particles))

        # Initialize filter settings
        self.n_particles = number_of_particles
        self.particles = []

        # State related settings
        self.state_dimension = 3  # x, y, theta
        self.x_min = limits[0]
        self.x_max = limits[1]
        self.y_min = limits[0]
        self.y_max = limits[1]

        # Set noise
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

        # Set SIR specific properties
        self.resampling_algorithm = resampling_algorithm
        self.resampler = Resampler()
        
    def initialize_particles_uniform(self):
        """
        Initialize the particles uniformly over the world assuming a 3D state (x, y, heading). No arguments are required
        and function always succeeds hence no return value.
        """

        # Initialize particles with uniform weight distribution
        self.particles = []
        weight = 1.0 / self.n_particles
        for i in range(self.n_particles):
            # Add particle i
            self.particles.append(
                [weight, [
                    np.random.uniform(self.x_min, self.x_max, 1)[0],
                    np.random.uniform(self.y_min, self.y_max, 1)[0],
                    np.random.uniform(0, 2 * np.pi, 1)[0]]
                 ]
            )

    @staticmethod
    
    def normalize_weights(weighted_samples):
        """
        Normalize all particle weights.
            """
        sum_weights = 0.0
        normalize_particle = []

        # TO DO : Compute sum weighted samples

        # TO DO : Compute normalized weights

        return normalize_particle

    def motion_model(self, sample, forward_motion, angular_motion):
        """
        Propagate an individual sample with a simple motion model that assumes the robot rotates angular_motion rad and
        then moves forward_motion meters in the direction of its heading. Return the propagated sample (leave input
        unchanged).

        :param sample: Sample (unweighted particle) that must be propagated
        :param forward_motion: Forward motion in meters
        :param angular_motion: Angular motion in radians
        :return: propagated sample
        """
        # 1. rotate by given amount plus additive noise sample (index 1 is angular noise standard deviation)
        updated_particle = copy.deepcopy(sample)
        updated_particle[2] += np.random.normal(angular_motion, self.process_noise[1], 1)[0]

        # Compute forward motion by combining deterministic forward motion with additive zero mean Gaussian noise
        forward_displacement = np.random.normal(forward_motion, self.process_noise[0], 1)[0]

        # TO DO : move forward


        return updated_particle
    
    def compute_particle_weights(self, sample, measurement, landmarks):
        """
        Compute particle weights p(z|sample) for a specific measurement given sample state and landmarks.

        :param sample: Sample (unweighted particle) that must be propagated
        :param measurement: List with measurements, for each landmark [distance_to_landmark, angle_wrt_landmark], units
        are meters and radians
        :param landmarks: Positions (absolute) landmarks (in meters)
        :return Likelihood
        """

        # Initialize measurement likelihood
        particle_weights = 1.0

        # Loop over all landmarks for current particle
        for i, lm in enumerate(landmarks):
            # Compute expected measurement assuming the current particle state
            dx = sample[0] - lm[0]
            dy = sample[1] - lm[1]
            expected_distance = np.sqrt(dx*dx + dy*dy)
            expected_angle = np.arctan2(dy, dx)

            # TO DO : Map difference true and expected distance measurement to probability
            p_z_given_x_distance = 

            # TO DO : Map difference true and expected angle measurement to probability
            p_z_given_x_angle = 

            # Incorporate likelihoods current landmark
            particle_weights *= p_z_given_x_distance * p_z_given_x_angle

        # Return importance weight based on all landmarks
        return particle_weights


    def update(self, robot_forward_motion, robot_angular_motion, measurements, landmarks):
        """
        Process a measurement given the measured robot displacement and resample if needed.

        :param robot_forward_motion: Measured forward robot motion in meters.
        :param robot_angular_motion: Measured angular robot motion in radians.
        :param measurements: Measurements.
        :param landmarks: Landmark positions.
        """

        # Loop over all particles
        new_particles = []
        for par in self.particles:
            # 1. Propagate the particle state according to the current particle
            motion_state = self.motion_model(par[1], robot_forward_motion, robot_angular_motion)

            # 2. Compute current particle's weight
            weight = par[0] * self.compute_particle_weights(motion_state, measurements, landmarks)

            # 3. Store
            new_particles.append([weight, motion_state])

        # 4. Update particles
        self.particles = self.normalize_weights(new_particles)
        
        # 5. Resample
        self.particles = self.resampler.resample(self.particles, self.n_particles, self.resampling_algorithm)
