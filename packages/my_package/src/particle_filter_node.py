#!/usr/bin/env python3

import os
import rospy
from sensor_msgs.msg import Range, Imu
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float64MultiArray, Header, Float64

import numpy as np
import math
from scipy.stats import multivariate_normal
import random
import copy


N = 100 # number of particles
update_freq = 5 # PF update frequency
dt = 1.0/update_freq # time interval

class ParticleFilterNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ParticleFilterNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        rospy.loginfo(f"INITIALIZING PF")

        # initialize measurements to 0
        # format: [z_x, z_y, z_theta]
        self.measurements = [0.0 for _ in range(3)]
        self.v = 0
        self.w = 0

        # subscribe to sensors
        self.tof_y_subscriber = rospy.Subscriber(f'{self._vehicle_name}/front_center_tof_driver_node/range', Range, self.tof_y_callback)
        self.tof_x_subscriber = rospy.Subscriber('tof_node_right/range', Range, self.tof_x_callback)
        self.angle_subscriber = rospy.Subscriber('vectornav/IMU', Imu, self.angle_callback)


        # subscribe to control input
        input_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.input_subscriber = rospy.Subscriber(input_topic, Twist2DStamped, self.input_callback)

        # publish state output
        self.state_pub = rospy.Publisher(f"{node_name}/state_est", Float64MultiArray, queue_size=1)

        # list of N default Particle objects
        self.particles = [Particle() for _ in range(N)]

        self.state = [0, 0, 0] # initial state (x, y, theta)

    def run(self):
        # publish message every dt seconds (or update_freq Hz)
        rate = rospy.Rate(update_freq)
        rate2 = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate2.sleep()
            print()
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

        # take yaw
        self.measurements[2] = euler_angles[2]
        # print(f"yaw measurement {euler_angles[2]}")
        return
    
    def input_callback(self, data):
        self.v = data.v
        self.w = data.omega

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

        # print(f"overall state: {self.state}")
    

        self.state_pub.publish(msg)
        
        return

    def propagate_particles(self):
        """propagate the state of each particle"""

        # print("pre propogate")
        # i = 0
        # for p in self.particles:
        #     print(f"PARTICLE {i}")
        #     print(f"x: {p.x}, y: {p.y}, theta: {p.theta}, weight:{p.weight}")
        #     i += 1

        # for particle in self.particles:
        #     particle.propagate_state(self.v, self.w)
        self.particles = [p.propagate_state(self.v, self.w) for p in self.particles]

        # print("post propogate")
        # i = 0
        # for p in self.particles:
        #     print(f"PARTICLE {i}")
        #     print(f"x: {p.x}, y: {p.y}, theta: {p.theta}, weight:{p.weight}")
        #     i += 1

        # i = 0
        # for p in self.particles:
        #     print(f"PARTICLE {i}")
        #     print(f"x: {p.x}, y: {p.y}, theta: {p.theta}, weight:{p.weight}")
        #     i += 1

        return

    def weight_particles(self):
        """assign weight to each particle, normalize at the end"""

        # print("pre weight")
        # i = 0
        # for p in self.particles:
        #     print(f"PARTICLE {i}")
        #     print(f"x: {p.x}, y: {p.y}, theta: {p.theta}, weight:{p.weight}")
        #     i += 1        

        # sample from distribution, get preliminary weights
        # for particle in self.particles:
        #     particle.assign_weight(self.measurements)
        self.particles = [p.assign_weight(self.measurements) for p in self.particles]



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

        # print("pre sample")
        # i = 0
        # for p in self.particles:
        #     print(f"PARTICLE {i}")
        #     print(f"x: {p.x}, y: {p.y}, theta: {p.theta}, weight:{p.weight}")
        #     i += 1


        
        # choose N particles from self.particles according to the weights in weights_vec
        self.particles = random.choices(population = self.particles, weights=weights_vec, k=N)

        particles_copy = [copy.deepcopy(p) for p in self.particles]

        self.particles = particles_copy

        # print("post sample")

        # i = 0
        # for p in self.particles:
        #     print(f"PARTICLE {i}")
        #     print(f"x: {p.x}, y: {p.y}, theta: {p.theta}, weight:{p.weight}")
        #     i += 1
        
        return

    def get_avg_particle(self):
        x = 0
        y = 0
        theta = 0

        # renormalize weights after resample step
        weights_total = sum([p.weight for p in self.particles])


        for particle in self.particles:
            x += particle.x * particle.weight / weights_total
            y += particle.y * particle.weight / weights_total
            theta += particle.theta * particle.weight / weights_total

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

        # print("noise")
        # print(noise_x)
        # print(noise_y)
        # print(noise_theta)

        # print(self.x)

        # run simple motion model
        self.x = self.x + v*dt*np.cos(self.theta) + noise_x
        self.y = self.y + v*dt*np.sin(self.theta) + noise_y
        self.theta = self.angle_wrap(self.theta + w*dt + noise_theta)

        # print(self.x)

        return self
    
    def assign_weight(self, measurements):
        tof_x_i, tof_y_i = self.get_theoretical_tof(self.theta, self.x, self.y)
        state_theo = np.array([tof_x_i, tof_y_i, self.theta])
        tof_actual = np.array(measurements) # get x and y tof measurements, IMU theta measurement
        tof_covariance = np.array([[0.05*tof_actual[0], 0, 0], [0, 0.05*tof_actual[1], 0], [0, 0, 0.3]]) # assume covariance is about 5% of measurement, taken from ToF datasheet
        # print(f"x: {self.x}, y: {self.y}, theta: {self.theta}")
        # print(f"tof theo: {state_theo}, tof_actual: {tof_actual}, cov: {tof_covariance}, weight: {self.weight}")

        try:
            self.weight = multivariate_normal.pdf(state_theo, tof_actual, tof_covariance) # P(z | x) = P(x | z) * P(z) / P(x) = normalization*P(x|z)
            # print(f"Weight passed thru pdf: {self.weight}")
        except:
            # print("weight pdf got messed up")
            pass

        return self

    def get_theoretical_tof(self, theta, x, y):
        """
        Return theoretical ToF values
        """

        x_tof = 0 # predicted x tof
        y_tof = 0 # predicted y tof

        L = 1 # box side length
        x_offset = 0.05 # sensor x offset from center
        y_offset = 0.07 # sensor y offset from center

        # x calculation

        if theta >= 0 and theta <= math.pi/2: # 0 to 90 degrees

            if y + math.tan(theta+3*math.pi/2)*(L/2-x) > -L/2: # projection above bottom wall - RIGHT WALL - case is good!
                x_tof = (L/2-x)/math.cos(theta-math.pi/2)-x_offset
            else: # projection below bottom wall - BOTTOM WALL - case is good!
                x_tof = (L/2+y)/math.cos(-theta)-x_offset

        elif theta >= math.pi/2 and theta <= math.pi:  # 90 to 180 degrees

            if y + math.tan(theta-math.pi/2)*(L/2-x) < L/2: # projection below top wall - RIGHT WALL - case is good!
                x_tof = (L/2-x)/math.cos(theta-math.pi/2)-x_offset
            else: # projection above top wall - TOP WALL - case is good!
                x_tof = (L/2-y)/math.cos(math.pi-theta)-x_offset

        elif theta >= math.pi and theta <= 3*math.pi/2:  # 180 to 270 degrees

            if y + math.tan(3*math.pi/2-theta)*(L/2+x) < L/2: # projection below top wall - LEFT WALL - case is good!
                x_tof = (L/2+x)/math.cos(3*math.pi/2-theta)-x_offset
            else: # projection above top wall - TOP WALL - case is good!
                x_tof = (L/2-y)/math.cos(math.pi-theta)-x_offset

        else: # 270 to 360 degrees

            if y + math.tan(3*math.pi/2-theta)*(L/2+x) > -L/2: # projection above bottom wall - LEFT WALL - case is good!
                x_tof = (L/2+x)/math.cos(3*math.pi/2-theta)-x_offset
            else: # projection below bottom wall - BOTTOM WALL - case is good!
                x_tof = (L/2+y)/math.cos(-theta)-x_offset

        theta += math.pi/2

        if theta >= 2*math.pi:
            theta -= 2*math.pi

        # y calculation

        if theta >= 0 and theta <= math.pi/2: # 0 to 90 degrees

            if y + math.tan(theta+3*math.pi/2)*(L/2-x) > -L/2: # projection above bottom wall - RIGHT WALL - case is good!
                y_tof = (L/2-x)/math.cos(theta-math.pi/2)-y_offset
            else: # projection below bottom wall - BOTTOM WALL - case is good!
                y_tof = (L/2+y)/math.cos(-theta)-y_offset

        elif theta >= math.pi/2 and theta <= math.pi:  # 90 to 180 degrees

            if y + math.tan(theta-math.pi/2)*(L/2-x) < L/2: # projection below top wall - RIGHT WALL - case is good!
                y_tof = (L/2-x)/math.cos(theta-math.pi/2)-y_offset
            else: # projection above top wall - TOP WALL - case is good!
                y_tof = (L/2-y)/math.cos(math.pi-theta)-y_offset

        elif theta >= math.pi and theta <= 3*math.pi/2:  # 180 to 270 degrees

            if y + math.tan(3*math.pi/2-theta)*(L/2+x) < L/2: # projection below top wall - LEFT WALL - case is good!
                y_tof = (L/2+x)/math.cos(3*math.pi/2-theta)-y_offset
            else: # projection above top wall - TOP WALL - case is good!
                y_tof = (L/2-y)/math.cos(math.pi-theta)-y_offset

        else: # 270 to 360 degrees

            if y + math.tan(3*math.pi/2-theta)*(L/2+x) > -L/2: # projection above bottom wall - LEFT WALL - case is good!
                y_tof = (L/2+x)/math.cos(3*math.pi/2-theta)-y_offset
            else: # projection below bottom wall - BOTTOM WALL - case is good!
                y_tof = (L/2+y)/math.cos(-theta)-y_offset

        return x_tof, y_tof



    def angle_wrap(self, angle):
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