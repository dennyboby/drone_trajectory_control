#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

# TUNEABLE PARAMETERS:
Kp = 85
Kd = 8
lambd_z = 4
lambd_roll = 7
lambd_pitch = 7
lambd_yaw = 4
k_z = 7
k_roll = 100
k_pitch = 100
k_yaw = 25

class Quadrotor():
    def __init__(self):
        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.mutex_lock_on = False
        rospy.on_shutdown(self.save_data)
        # TODO: include initialization codes if needed
        self.waypoints = np.array([[0,0,0], [0,0,1], [1,0,1], [1,1,1], [0,1,1], [0,0,1]]) # Each row has x y z
        self.waypoints_time = np.array([0, 5, 20, 35, 50, 65])
        self.vel = np.array([0, 0, 0])
        self.acc = np.array([0, 0, 0])
        self.mass = 0.027
        self.g = 9.81
        self.kF = 1.28192*(10**(-8))
        self.kM = 5.964552*(10**(-3))
        self.l = 0.046
        self.Ix = 16.5717*(10**(-6))
        self.Iy = 16.5717*(10**(-6))
        self.Iz = 29.261652*(10**(-6))
        self.Ip = 12.65625*(10**(-8))
        self.rotor_max = 2618
        self.rotor_min = 0
        # Initial u
        self.u1 = 0.01
        self.u2 = 0.01
        self.u3 = 0.01
        self.u4 = 0.01

        self.w = self.motor_speeds(self.u1, self.u2, self.u3, self.u4)
        self.w1 = self.w[0]
        self.w2 = self.w[1]
        self.w3 = self.w[2]
        self.w4 = self.w[3]


    def quinticpoly(self, t0, tf, q0, q0_dot, q0_ddot, qf, qf_dot, qf_ddot):
        b = np.array([q0, q0_dot, q0_ddot, qf, qf_dot, qf_ddot])
        A = np.array([[1, t0, t0**2,   t0**3,    t0**4,    t0**5],
                      [0,  1,  2*t0, 3*t0**2,  4*t0**3,  5*t0**4],
                      [0,  0,     2,    6*t0, 12*t0**2, 20*t0**3],
                      [1, tf, tf**2,   tf**3,    tf**4,    tf**5],
                      [0,  1,  2*tf, 3*tf**2,  4*tf**3,  5*tf**4],
                      [0,  0,     2,    6*tf, 12*tf**2, 20*tf**3]])
        a = np.matmul(np.linalg.inv(A),b)
        return a

    def motor_speeds(self, u1, u2, u3, u4):
        AM = np.array([ [1/(4*self.kF),  -sqrt(2)/(4*self.kF*self.l),   -sqrt(2)/(4*self.kF*self.l),   -1/(4*self.kM*self.kF)],
                        [1/(4*self.kF),  -sqrt(2)/(4*self.kF*self.l),   sqrt(2)/(4*self.kF*self.l),    1/(4*self.kM*self.kF)],
                        [1/(4*self.kF),  sqrt(2)/(4*self.kF*self.l),    sqrt(2)/(4*self.kF*self.l),    -1/(4*self.kM*self.kF)],
                        [1/(4*self.kF),  sqrt(2)/(4*self.kF*self.l),    -sqrt(2)/(4*self.kF*self.l),   1/(4*self.kM*self.kF)]       
                        ])
        U = np.array([u1, u2, u3, u4])
        return (np.sqrt(self.only_pos(np.dot(AM,U))))

    def limit_speed(self, speed, lower_limit, upper_limit):
        if (speed <= lower_limit):
            speed = lower_limit
        elif (speed > upper_limit):
            speed = upper_limit
        return speed

    # Saturation function
    def sat(self, surface, a=0.2):
        return min(max(surface/a,-1),1)

    def only_pos(self, z):
        return(np.maximum(0,z))

    def wrap_to_pi(self, angle):
        angle = atan2(sin(angle), cos(angle))
        return angle 

    def generate_trajectory(self, t0, tf, pos_i, vel_i, acc_i, pos_f, vel_f, acc_f):
        x_coeff = self.quinticpoly(t0, tf, pos_i[0], vel_i[0], acc_i[0], pos_f[0], vel_f[0], acc_f[0])
        y_coeff = self.quinticpoly(t0, tf, pos_i[1], vel_i[1], acc_i[1], pos_f[1], vel_f[1], acc_f[1])
        z_coeff = self.quinticpoly(t0, tf, pos_i[2], vel_i[2], acc_i[2], pos_f[2], vel_f[2], acc_f[2])
        qd = np.array([[x_coeff[0] + x_coeff[1]*self.t + x_coeff[2]*self.t**2 + x_coeff[3]*self.t**3 + x_coeff[4]*self.t**4 + x_coeff[5]*self.t**5],
                       [y_coeff[0] + y_coeff[1]*self.t + y_coeff[2]*self.t**2 + y_coeff[3]*self.t**3 + y_coeff[4]*self.t**4 + y_coeff[5]*self.t**5],
                       [z_coeff[0] + z_coeff[1]*self.t + z_coeff[2]*self.t**2 + z_coeff[3]*self.t**3 + z_coeff[4]*self.t**4 + z_coeff[5]*self.t**5]])
        qd_dot = np.array([[x_coeff[1] + 2*x_coeff[2]*self.t + 3*x_coeff[3]*self.t**2 + 4*x_coeff[4]*self.t**3 + 5*x_coeff[5]*self.t**4],
                           [y_coeff[1] + 2*y_coeff[2]*self.t + 3*y_coeff[3]*self.t**2 + 4*y_coeff[4]*self.t**3 + 5*y_coeff[5]*self.t**4],
                           [z_coeff[1] + 2*z_coeff[2]*self.t + 3*z_coeff[3]*self.t**2 + 4*z_coeff[4]*self.t**3 + 5*z_coeff[5]*self.t**4]])
        qd_ddot = np.array([[2*x_coeff[2] + 6*x_coeff[3]*self.t + 12*x_coeff[4]*self.t**2 + 20*x_coeff[5]*self.t**3],
                            [2*y_coeff[2] + 6*y_coeff[3]*self.t + 12*y_coeff[4]*self.t**2 + 20*y_coeff[5]*self.t**3],
                            [2*z_coeff[2] + 6*z_coeff[3]*self.t + 12*z_coeff[4]*self.t**2 + 20*z_coeff[5]*self.t**3]])
        return qd, qd_dot, qd_ddot

    def traj_evaluate(self):
        # TODO: evaluating the corresponding trajectories designed in Part 1 to return the desired positions, velocities and accelerations
        desired_p = np.array([[0],[0],[0]])
        desired_v = np.array([[0],[0],[0]])
        desired_a = np.array([[0],[0],[0]])
        print("Time: ", self.t)
        if self.t == None:
            print("Time is None")
            return desired_p, desired_v, desired_a
        if self.t <= self.waypoints_time[1]:
            desired_p, desired_v, desired_a = self.generate_trajectory(self.waypoints_time[0], self.waypoints_time[1], self.waypoints[0], self.vel, self.acc, self.waypoints[1], self.vel, self.acc)
        elif self.t <= self.waypoints_time[2]:
            desired_p, desired_v, desired_a = self.generate_trajectory(self.waypoints_time[1], self.waypoints_time[2], self.waypoints[1], self.vel, self.acc, self.waypoints[2], self.vel, self.acc)
        elif self.t <= self.waypoints_time[3]:
            desired_p, desired_v, desired_a = self.generate_trajectory(self.waypoints_time[2], self.waypoints_time[3], self.waypoints[2], self.vel, self.acc, self.waypoints[3], self.vel, self.acc)
        elif self.t <= self.waypoints_time[4]:
            desired_p, desired_v, desired_a = self.generate_trajectory(self.waypoints_time[3], self.waypoints_time[4], self.waypoints[3], self.vel, self.acc, self.waypoints[4], self.vel, self.acc)
        elif self.t <= self.waypoints_time[5]:
            desired_p, desired_v, desired_a = self.generate_trajectory(self.waypoints_time[4], self.waypoints_time[5], self.waypoints[4], self.vel, self.acc, self.waypoints[5], self.vel, self.acc)
        else:
            print("Time exceeds %d seconds" %self.waypoints_time[5])
        return desired_p, desired_v, desired_a

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        # obtain the desired values by evaluating the corresponding trajectories
        d_pos, d_vel, d_acc = self.traj_evaluate()
        # TODO: implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"
        # REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]
        Fx = self.mass*(-Kp*(xyz[0,0] - d_pos[0,0]) - Kd*(xyz_dot[0,0] - d_vel[0,0]) + d_acc[0,0])
        Fy = self.mass*(-Kp*(xyz[1,0] - d_pos[1,0]) - Kd*(xyz_dot[1,0] - d_vel[1,0]) + d_acc[1,0])

        d_roll = np.arcsin(-Fy/self.u1)
        d_pitch = np.arcsin(Fx/self.u1)
        d_yaw = 0

        d_droll = d_dpitch = d_dyaw = 0
        d_ddroll = d_ddpitch = d_ddyaw = 0

        error_z = xyz[2,0] - d_pos[2,0]
        error_roll = self.wrap_to_pi(rpy[0,0] - d_roll)
        error_pitch = self.wrap_to_pi(rpy[1,0] - d_pitch)
        error_yaw = self.wrap_to_pi(rpy[2,0] - d_yaw)

        derror_z = xyz_dot[2,0] - d_vel[2,0]
        derror_roll = rpy_dot[0,0] - d_droll
        derror_pitch = rpy_dot[1,0] - d_dpitch
        derror_yaw = rpy_dot[2,0] - d_dyaw

        omega = self.w1 - self.w2 + self.w3 - self.w4
        
        surface_z = derror_z + lambd_z*error_z
        surface_roll = derror_roll + lambd_roll*error_roll
        surface_pitch = derror_pitch + lambd_pitch*error_pitch
        surface_yaw = derror_yaw + lambd_yaw*error_yaw

        self.u1 = (self.mass *(self.g + d_acc[2,0] - lambd_z*derror_z - k_z*self.sat(surface_z, a=1) ) ) / (cos(rpy[0,0])*cos(rpy[1,0]))
        self.u2 =  (-rpy_dot[1,0]*rpy_dot[2,0]*(self.Iy - self.Iz)) + (self.Ip*omega*rpy_dot[1,0]) + (self.Ix*d_ddroll) - (lambd_roll*self.Ix*derror_roll) - (self.Ix*k_roll*self.sat(surface_roll)) 
        self.u3 = (-rpy_dot[0,0]*rpy_dot[2,0]*(self.Iz-self.Ix)) - (self.Ip*omega*rpy_dot[0,0]) + (self.Iy*d_ddpitch) - (lambd_pitch*self.Iy*derror_pitch) - (self.Iy*k_pitch*self.sat(surface_pitch))
        self.u4 = (-rpy_dot[0,0]*rpy_dot[1,0]*(self.Ix-self.Iy)) + (self.Iz*d_ddyaw) - (lambd_yaw*self.Iz*derror_yaw) - (k_yaw*self.Iz*self.sat(surface_yaw))
        # TODO: convert the desired control inputs "u" to desired rotor velocities "motor_vel" by using the "allocation matrix"
        motor_vel = self.motor_speeds(self.u1, self.u2, self.u3, self.u4)
        # TODO: maintain the rotor velocities within the valid range of [0 to 2618]
        motor_vel[0] = self.limit_speed(motor_vel[0], self.rotor_min, self.rotor_max)
        motor_vel[1] = self.limit_speed(motor_vel[1], self.rotor_min, self.rotor_max)
        motor_vel[2] = self.limit_speed(motor_vel[2], self.rotor_min, self.rotor_max)
        motor_vel[3] = self.limit_speed(motor_vel[3], self.rotor_min, self.rotor_max)
        # publish the motor velocities to the associated ROS topic
        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0], motor_vel[1], motor_vel[2], motor_vel[3]]
        self.motor_speed_pub.publish(motor_speed)

    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([
        [1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
        [0, np.cos(rpy[0]), -np.sin(rpy[0])],
        [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
        ]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

    # save the actual trajectory data
    def save_data(self):
        # TODO: update the path below with the correct path
        with open("log.pkl", "wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)

if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")