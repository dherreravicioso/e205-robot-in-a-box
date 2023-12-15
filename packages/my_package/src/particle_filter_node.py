#!/usr/bin/env python3

import os
import rospy
from sensor_msgs.msg import Range, IMU
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float64MultiArray, Header

import numpy as np
import math
from scipy.stats import multivariate_normal
import random


N = 10 # number of particles
update_freq = 10 # PF update frequency
dt = 1/update_freq # time interval

class ParticleFilterNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ParticleFilterNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        print(f"INITIALIZING PF")

        # initialize measurements to 0
        # format: [z_x, z_y, z_theta]
        self.measurements = [0 for _ in range(3)]
        self.v = 0
        self.w = 0

        # subscribe to sensors
        self.tof_y_subscriber = rospy.Subscriber(f'{self._vehicle_name}/front_center_tof_driver_node/range', Range, self.tof_x_callback)
        self.tof_x_subscriber = rospy.Subscriber('right_side_tof/range', Range, self.tof_y_callback)
        self.angle_subscriber = rospy.Subscriber('vectornav/IMU', IMU, self.angle_callback)

        # subscribe to control input
        input_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.input_subscriber = rospy.Subscriber(input_topic, Twist2DStamped, self.input_callback)

        # publish state output
        self.state_pub = rospy.Publisher(f"{self._vehicle_name}/state_est", Float64MultiArray, queue_size=1)

        # list of N default Particle objects
        self.particles = [Particle() for _ in range(N)]

        self.state = [0, 0, 0] # initial state (x, y, theta)

    def run(self):
        # publish message every dt seconds (or update_freq Hz)
        rate = rospy.Rate(dt)
        while not rospy.is_shutdown():
            self.propagate_particles()
            self.weight_particles()
            self.resample_particles()
            self.get_avg_particle()
            self.publish_state()

            rate.sleep()

    def tof_x_callback(self, data):
        self.measurements[0] = data.range
        return

    def tof_y_callback(self, data):
        self.measurements[1] = data.range
        return

    def angle_callback(self, data):
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w

        euler_angles = self.quat_to_euler(x, y, z, w)
        print(f"angles are {euler_angles}")

        # take yaw
        self.measurements[2] = euler_angles[2]
        return
    
    def input_callback(self, data):
        self.v = data.v
        self.w = data.omega

        print(f"v={self.v}, w={self.w}")

    def quat_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
    def publish_state(self):

        msg = Float64MultiArray(
            data = self.state
        )

        self.state_pub.publish(msg)
        
        return

    def propagate_particles(self):
        """propagate the state of each particle"""
        for particle in self.particles:
            particle.propagate_state()

        return

    def weight_particles(self):
        """assign weight to each particle, normalize at the end"""
        
        # sample from distribution, get preliminary weights
        for particle in self.particles:
            particle.assign_weight()


        # get total sum of weights
        weight_sum = 0
        for particle in self.particles:
            weight_sum += particle.weight
        
        # normalize
        for particle in self.particles:
            # deal with case where weights are very small, weight equally and hope for the best
            if weight_sum == 0:
                particle.weight = 1/N
            else:
                particle.weight = particle.weight/weight_sum

        return
        

    def resample_particles(self):
        weights_vec = [p.weight for p in self.particles]

        # choose N particles from self.particles according to the weights in weights_vec
        resampled_particles = random.choices(population = self.particles, weights=weights_vec, k=N)

        self.particles = resampled_particles

        return

    def get_avg_particle(self):
        x = 0
        y = 0
        theta = 0

        for particle in self.particles:
            x += particle.x * particle.weight
            y += particle.y * particle.weight
            theta += particle.theta * particle.weight

        self.state = [x, y, theta]

        return


    

class Particle():
    """
    Particle class to hold pose info and propagation functions
    """

    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.weight = 1/N

        self.noise_var_pos = 0.1
        self.noise_var_theta = 0.1
        
    
    def propagate_state(self, v, w):
        """
        Propagates state based on input velocity v and input angular velocity w
        """

        # create additive noise
        noise_x = np.random.normal(loc=0.0, scale=self.noise_var_pos, size=None)
        noise_y = np.random.normal(loc=0.0, scale=self.noise_var_pos, size=None)
        noise_theta = np.random.normal(loc=0.0, scale=self.noise_var_theta, size=None)

        # run simple motion model
        self.x = self.x + v*dt*np.cos(self.theta) + noise_x
        self.y = self.y + v*dt*np.sin(self.theta) + noise_y
        self.theta = self.angle_wrap(self.theta + w*dt + noise_theta)

        return
    
    def assign_weight(self, measurements):
        tof_x_i, tof_y_i = self.get_theoretical_tof()
        tof_theo = np.array([tof_x_i, tof_y_i])
        tof_actual = np.array(measurements[0:2]) # get x and y tof measurements
        tof_covariance = np.array([[0.05*tof_actual[0], 0], [0, 0.05*tof_actual[1]]]) # assume covariance is about 5% of measurement, taken from ToF datasheet

        self.weight = multivariate_normal.pdf(tof_theo, tof_actual, tof_covariance) # P(z | x) = P(x | z) * P(z) / P(x) = normalization*P(x|z)

        return

    def get_theoretical_tof(self):
        """
        Return theoretical ToF values as tuple:
        tof_x, tof_y
        """
        return 1, 1

    
    def angle_wrap(self, angle):
        """Wrap angle data in radians to [-pi, pi]

        Parameters:
        angle (float)   -- unwrapped angle

        Returns:
        angle (float)   -- wrapped angle
        """

        while angle >= np.pi:
            angle -= 2*np.pi

        while angle <= -np.pi:
            angle += 2*np.pi
        return angle







if __name__ == '__main__':
    # create the node
    node = ParticleFilterNode(node_name='particle_filter_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()