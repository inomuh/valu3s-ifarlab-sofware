#!/usr/bin/env python3

import numpy as np
import time

import rospy
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Kalman:
    def __init__(self):
        rospy.init_node("odom_combined")
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("ota_ir",Range, self.ir_callback)
        self.ir_origin = 2.4988601207733154
        self.main()
    
    def main(self):
        rate = rospy.Rate(5)
        pre_time = time.time()
        self.state = None
        # Save the Initial States
        vehicle_position_history = [[0,0]]
        vehicle_velocity_history = [[0,0]]
        measurement_history = [None]
        estimated_error_history = [None]
        self.ir_position = [0,0]
        self.odom = [0,0,0,0,0]
        #self.state = np.array([0,0,0,0])
        #self.covariance = np.diag(np.array([init_pos_std*init_pos_std,
        #                                    init_pos_std*init_pos_std,
        #                                    init_vel_std*init_vel_std,
        #                                    init_vel_std*init_vel_std]))

        # Setup the Model F Matrix

        # Set the Q Matrix
        accel_std = 0.1

        # Setup the Model H Matrix
        self.H = np.array([[1,0,0,0],[0,1,0,0]])

        # Set the R Matrix
        meas_std = 10.0
        self.R = np.diag([meas_std*meas_std, meas_std*meas_std])

        measurement_innovation_history = [None]
        measurement_innovation_covariance_history = [None]

        while not rospy.is_shutdown():
            step = time.time()
            dt = step - pre_time
            print("delta_t",dt)
  
            # Create the Simulation Objects

            # Set Initial State and Covariance (COMMENT OUT FOR DELAYED INIT)
            init_pos_std = 100
            init_vel_std = 100
            # KF Measurement
            if self.ir_position != 0:
                measurement = [self.ir_position[0],self.ir_position[1]]

                if self.state is not None and self.covariance is not None:

                    # KF Prediction
        #         kalman_filter.prediction_step()
                    # Make Sure Filter is Initialised
                    x = self.state
                    P = self.covariance
                    H = self.H
                    R = self.R
                    self.Q = np.diag(np.array([(0.5*dt*dt),(0.5*dt*dt),dt,dt]) * (accel_std*accel_std))
                    self.F = np.array([[1,0,dt,0],
                                    [0,1,0,dt],
                                    [0,0,1,0],
                                    [0,0,0,1]])      
                    # Calculate Kalman Filter Prediction
                    x_predict = np.matmul(self.F, x) 
                    P_predict = np.matmul(self.F, np.matmul(P, np.transpose(self.F))) + self.Q

                    # Save Predicted State
                    self.state = x_predict
                    self.covariance = P_predict

        #            kalman_filter.update_step(measurement)

                    # Calculate Kalman Filter Update
                    z = np.array([measurement[0],measurement[1]])  # ir position
                    z_hat = np.matmul(H, x)

                    y = z - z_hat
                    S = np.matmul(H,np.matmul(P,np.transpose(H))) + R
                    K = np.matmul(P,np.matmul(np.transpose(H),np.linalg.inv(S)))
                            
                    x_update = x + np.matmul(K, y)
                    P_update = np.matmul( (np.eye(4) - np.matmul(K,H)), P)

                    print(x_update[0],x_update[1],x_update[2],x_update[3])

                    # Save Updated State
                    self.innovation = y
                    self.innovation_covariance = S
                    self.state = x_update
                    self.covariance = P_update

                    measurement_innovation_history.append(self.innovation)
                    measurement_innovation_covariance_history.append(self.innovation_covariance)

                    # Estimation Error
                    estimation_error = None
                    estimated_state = self.state
                    if estimated_state is not None:
                        estimation_error = [estimated_state[0] - measurement[0], 
                                            estimated_state[1] - measurement[1],
                                            estimated_state[2] - self.odom[3], 
                                            estimated_state[3] - self.odom[4]]

                    # Save Data
                    vehicle_position_history.append(measurement)
                    vehicle_velocity_history.append([self.odom[3],self.odom[4]])
                    measurement_history.append(measurement)
                    estimated_error_history.append(estimation_error)

                    # Calculate Stats
                    x_innov_std = np.std([v[0] for v in measurement_innovation_history if v is not None])
                    y_innov_std = np.std([v[1] for v in measurement_innovation_history if v is not None])
                    pos_mse = np.mean([(v[0]**2+v[1]**2) for v in estimated_error_history if v is not None])
                    vel_mse = np.mean([(v[2]**2+v[3]**2) for v in estimated_error_history if v is not None])
                    print('X Position Measurement Innovation Std: {} (m)'.format(x_innov_std))
                    print('Y Position Measurement Innovation Std: {} (m)'.format(y_innov_std))
                    print('Position Mean Squared Error: {} (m)^2'.format(pos_mse))
                    print('Velocity Mean Squared Error: {} (m/s)^2'.format(vel_mse))                
                
                else:
                    self.state = np.array([measurement[0],measurement[1],0,0])
                    # self.covariance = np.diag(np.array([self.R[0,0],self.R[1,1],init_vel_std*init_vel_std,init_vel_std*init_vel_std]))
                    self.covariance = np.diag(np.array([init_pos_std*init_pos_std,
                                                        init_pos_std*init_pos_std,
                                                        init_vel_std*init_vel_std,
                                                        init_vel_std*init_vel_std]))                    
                    print("else")
            pre_time = step                    
        rate.sleep()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x,
                                            orientation.y,
                                            orientation.z,
                                            orientation.w])
        twist = msg.twist.twist
        self.odom = [position.x,
                    position.y,
                    yaw,
                    twist.linear.x,
                    twist.angular.z]
        # print(self.odom)
    
    def ir_callback(self, msg):
        data = msg.range
        self.ir_position[0] = self.ir_origin - data
        self.ir_position[1] = 0

if __name__ == "__main__":
    Kalman()






