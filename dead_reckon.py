#!/usr/bin/env python3

import numpy as np
import dqrobotics as dq
from dead_step_class import DeadStep
from data_loader import LoadExperimentData


class DeadReckoning:
    def __init__(self):
        self.ds = DeadStep()
        self.r_hat_B_I_guess = (np.cos(-np.pi / 4) + dq.k_ * np.sin(-np.pi / 4)) * (
            np.cos(np.pi / 4) + dq.i_ * np.sin(np.pi / 4))
        experiment_number = 7
        self.data = LoadExperimentData(experiment_number)
        self.dvl_vel_data = self.data.dvl_velocities                # νD readings
        self.imu_ang_vel_data = self.data.imu_angular_velocities    # ωI readings
        self.imu_lin_acc_data = self.data.imu_linear_accelerations  # gI static component
        self.end = len(self.dvl_vel_data[0])                        # sample length



    def generate_dualQ(self):

        # Initial guess of IMU rotation WRT body based on observations recorded during experiment
        r_hat_B_I_kminus1 = self.r_hat_B_I_guess
        # Initial pose
        x_W_B_kminus1 = self.data.initial_pos  # x̂W_B[0]

        # Prepare outputs for dead reckoning path
        DR_x_and_y = np.zeros((2, (self.end - self.data.calibration_time + 1)))
        start_pt = dq.vec3(self.data.initial_pos.translation())
        DR_x_and_y[:, 0] = start_pt[:2]
        yaw = np.zeros(self.end - self.data.calibration_time + 1)
        yaw[0] = x_W_B_kminus1.rotation_angle()
        index_for_DR = 1

        # Dead reckoning loop (Algorithm 1)
        for k in range(0, self.end):
            # pull single occurrences of data from stored array
            g_I_k = [self.imu_lin_acc_data[0, k],
                     self.imu_lin_acc_data[1, k],
                     self.imu_lin_acc_data[2, k]]
            w_I = [self.imu_ang_vel_data[0, k],
                   self.imu_ang_vel_data[1, k],
                   self.imu_ang_vel_data[2, k]]
            v_D_k = [self.dvl_vel_data[0, k],
                     self.dvl_vel_data[1, k],
                     self.dvl_vel_data[2, k]]

            ######## IMU alignment/calibration phase (Algorithm 1 lines 7–10): ######
            r_hat_B_I_k = self.ds.rotation_estimator(k, g_I_k, r_hat_B_I_kminus1)
            r_hat_B_I_kminus1 = r_hat_B_I_k

            if k > self.data.calibration_time:
                ######## IMU+DVL fusion and pose update (Algorithm 1 lines 12–16):#######
                x_W_B_k = self.ds.velocity_pose_update(r_hat_B_I_k, w_I, v_D_k, x_W_B_kminus1)
                x_W_B_kminus1 = x_W_B_k

                # Record dead-reckoned position
                pt = dq.vec3(x_W_B_k.translation())
                DR_x_and_y[:, index_for_DR] = pt[:2]
                yaw[index_for_DR] = x_W_B_k.rotation_angle()
                index_for_DR += 1

        return DR_x_and_y
