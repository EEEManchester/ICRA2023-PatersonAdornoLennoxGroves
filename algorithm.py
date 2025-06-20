#!/usr/bin/env python

"""
This file is a Python port of an identically named MATLAB version.
Simple script to evaluate the data fusion algorithm from the IMU and DVL.
Result of weekly meetings between Jessica Paterson, Keir Groves, and Bruno Adorno.

Conventions:
 - Variables x_sup_sub represent x^{sup}_{sub} in LaTeX
 - Variables p_sup_sub1sub2 represent p^{sup}_{sub1,sub2} in LaTeX

Current assumptions for this simulation:
  1) The IMU, DVL, and body frames are aligned and located at the CoM.
  2) The world frame is defined as the body frame when k=0.

Warning about notation: We are using the discrete time k. Not to confuse
with the imaginary unit k_ !
"""
import numpy as np
import dqrobotics as dq
from dead_step_class import DeadStep

ds = DeadStep()


# Create an instance of the class
ds = DeadStep()

def generate_dualQ(
    data,
    calibration_time,
    r_hat_B_I_kminus1=dq.DQ([1]),
    freq=50,
    initial_pos=dq.DQ([1, 0, 0, 0, 0, 0, 0, 0]),
):
    """
    Main routine to perform dead reckoning using dual quaternions.
    Follows Algorithm 1 structure, mapping code blocks to steps:
      • Lines 1–2: Initialization (sampling time T, initial gravity and poses)
      • Lines 3–6: Sensor data acquisition in loop
      • Lines 7–10: IMU alignment/calibration phase
      • Lines 12–16: IMU+DVL fusion and pose update
    """
    dvl_vel_data = data.dvl_velocities  # νD readings
    imu_ang_vel_data = data.imu_angular_velocities  # ωI readings
    imu_lin_acc_data = data.imu_linear_accelerations  # gI static component

    # Known DVL-to-body rotation (r_B_D = 180° about x-axis)
    r_B_D = dq.i_
    # Initialize pose
    x_W_B_kminus1 = initial_pos  # x̂W_B[0]
    # Initialize imu gravity vector rolling average
    g_avg_I = dq.DQ([0])
    # Sampling time T (line 1)
    T = 1 / freq               
    # sample length
    end = len(dvl_vel_data[0])

    # Prepare outputs for dead reckoning path
    DR_x_and_y = np.zeros((2, (end - calibration_time + 1)))
    start_pt = dq.vec3(initial_pos.translation())
    DR_x_and_y[:, 0] = start_pt[:2]
    yaw = np.zeros(end - calibration_time + 1)
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
        gain = 10
        g_B = dq.k_
        # Normalize current IMU gravity vector measurement
        g_I_DQ = dq.DQ(g_I_k)
        g_I_normalised = dq.normalize(g_I_DQ)
        # Update zero-frequency (running average) gravity vector in IMU frame (line 7)
        g_avg_I = dq.DQ(dq.vec3(g_I_normalised) / (k + 1)) \
                  + (k / (k + 1)) * g_avg_I
        # Estimate gravity in body frame using current rotation (line 8)
        g_hat_B_k = dq.Ad(r_hat_B_I_kminus1, g_avg_I)
        # Compute correction angular velocity with gain λ=10 (line 9)
        w_hat_B_BI_k = gain * dq.cross(g_hat_B_k, g_B)
        # Update IMU-to-body rotation via exponential map (line 10)
        r_hat_B_I_k = dq.exp(0.5 * T * w_hat_B_BI_k) * r_hat_B_I_kminus1
        # move to next step

        r_hat_B_I_k = ds.rotation_estimator(k, g_I_k, r_hat_B_I_kminus1)
        r_hat_B_I_kminus1 = r_hat_B_I_k


        if k > calibration_time:
            ######## IMU+DVL fusion and pose update (Algorithm 1 lines 12–16):#######
            # Body-frame angular velocity ω̂W,B (line 12)
            w_hat_B_WB = dq.Ad(r_hat_B_I_k, dq.DQ(w_I))
            # DVL linear velocity projection (line 13)
            p_hat_dot_B_WB = dq.Ad(r_B_D, dq.DQ(v_D_k))
            # Combine angular + linear into dual twist ξ̂B_W,B (line 15)
            twist_hat_B_WB = w_hat_B_WB + dq.E_ * p_hat_dot_B_WB
            # Integrate pose with world-frame twist (line 16)
            x_W_B_k = x_W_B_kminus1 * dq.exp((T / 2) * twist_hat_B_WB)
            # move to next step
            x_W_B_kminus1 = x_W_B_k

            # Record dead-reckoned position
            pt = dq.vec3(x_W_B_k.translation())
            DR_x_and_y[:, index_for_DR] = pt[:2]
            yaw[index_for_DR] = x_W_B_k.rotation_angle()
            index_for_DR += 1

    return DR_x_and_y
