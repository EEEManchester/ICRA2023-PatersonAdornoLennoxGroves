#!/usr/bin/env python3

import numpy as np
import dqrobotics as dq
from dead_step_class import DeadStep

class DeadReckoning:
    def __init__(self):
        self.ds = DeadStep()

    def generate_dualQ(self, data):
        dvl_vel_data = data.dvl_velocities  # νD readings
        imu_ang_vel_data = data.imu_angular_velocities  # ωI readings
        imu_lin_acc_data = data.imu_linear_accelerations  # gI static component

        # Initial guess of IMU rotation WRT body based on observations recorded during experiment
        r_hat_B_I_kminus1 = (np.cos(-np.pi / 4) + dq.k_ * np.sin(-np.pi / 4)) * (
            np.cos(np.pi / 4) + dq.i_ * np.sin(np.pi / 4))
        # Initial pose
        x_W_B_kminus1 = data.initial_pos  # x̂W_B[0]

        # sample length
        end = len(dvl_vel_data[0])

        # Prepare outputs for dead reckoning path
        DR_x_and_y = np.zeros((2, (end - data.calibration_time + 1)))
        start_pt = dq.vec3(data.initial_pos.translation())
        DR_x_and_y[:, 0] = start_pt[:2]
        yaw = np.zeros(end - data.calibration_time + 1)
        yaw[0] = x_W_B_kminus1.rotation_angle()
        index_for_DR = 1

        # Dead reckoning loop (Algorithm 1)
        for k in range(0, end):
            # pull single occurrences of data from stored array
            g_I_k = [imu_lin_acc_data[0, k],
                     imu_lin_acc_data[1, k],
                     imu_lin_acc_data[2, k]]
            w_I = [imu_ang_vel_data[0, k],
                   imu_ang_vel_data[1, k],
                   imu_ang_vel_data[2, k]]
            v_D_k = [dvl_vel_data[0, k],
                     dvl_vel_data[1, k],
                     dvl_vel_data[2, k]]

            ######## IMU alignment/calibration phase (Algorithm 1 lines 7–10): ######
            r_hat_B_I_k = self.ds.rotation_estimator(k, g_I_k, r_hat_B_I_kminus1)
            r_hat_B_I_kminus1 = r_hat_B_I_k

            if k > data.calibration_time:
                ######## IMU+DVL fusion and pose update (Algorithm 1 lines 12–16):#######
                x_W_B_k = self.ds.velocity_pose_update(r_hat_B_I_k, w_I, v_D_k, x_W_B_kminus1)
                x_W_B_kminus1 = x_W_B_k

                # Record dead-reckoned position
                pt = dq.vec3(x_W_B_k.translation())
                DR_x_and_y[:, index_for_DR] = pt[:2]
                yaw[index_for_DR] = x_W_B_k.rotation_angle()
                index_for_DR += 1

        return DR_x_and_y
