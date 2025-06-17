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


def dead_reckoning_step(r_hat_B_I, r_B_D, w_I, v_D_k, x_prev, dt):
    """
    
    computes body-frame angular velocity, DVL projection, twist, and integrates pose.
    """
    # Body-frame angular velocity ω̂W,B (line 12)
    w_hat_B_WB = dq.Ad(r_hat_B_I, dq.DQ(w_I))
    # DVL linear velocity projection (line 13)
    p_hat_dot_B_WB = dq.Ad(r_B_D, dq.DQ(v_D_k))
    # Combine angular + linear into dual twist ξ̂B_W,B (line 15)
    twist_hat_B_WB = w_hat_B_WB + dq.E_ * p_hat_dot_B_WB
    # Integrate pose with world-frame twist (line 16)
    x_W_B_k = x_prev * dq.exp((dt / 2) * twist_hat_B_WB)
    return x_W_B_k


def rotation_estimate_step(g_I_k, k, g_avg_kminus1, r_hat_B_I_kminus1, dt):
    """
    Process one gravity vector measurement to update IMU-to-body rotation estimate.
    Corresponds to Algorithm 1 lines 7–10 in the paper:
    """
    gain = 10
    g_B=dq.k_
    # Normalize current IMU gravity vector measurement
    g_I_DQ = dq.DQ(g_I_k)
    g_I_normalised = dq.normalize(g_I_DQ)

    # Update zero-frequency (running average) gravity vector in IMU frame (line 7)
    g_avg_I_k = dq.DQ(dq.vec3(g_I_normalised) / (k + 1)) \
            + (k / (k + 1)) * g_avg_kminus1

    # Estimate gravity in body frame using current rotation (line 8)
    g_hat_B_k = dq.Ad(r_hat_B_I_kminus1, g_avg_I_k)

    # Compute correction angular velocity with gain λ=10 (line 9)
    w_hat_B_BI_k = gain * dq.cross(g_hat_B_k, g_B)

    # Update IMU-to-body rotation via exponential map (line 10)
    r_hat_B_I_k = dq.exp(0.5 * dt * w_hat_B_BI_k) * r_hat_B_I_kminus1

    ###### Compute and return gravity error norm - put this in if required ####
    # g_error = g_B_estimated - dq.k_
    # norm_g_error = dq.vec8(dq.norm(g_error))[0]

    return r_hat_B_I_k, g_avg_I_k


def generate_dualQ(
    data,
    calibration_time,
    r_hat_B_I_kminus1=dq.DQ([1]),
    freq=50,
    initial_pos=dq.DQ([1, 0, 0, 0, 0, 0, 0, 0]),
):
    """
    Main routine to perform dead reckoning using dual quaternions.
    Follows Algorithm 1 structure, mapping code blocks to steps:
      • Lines 1–2: Initialization (sampling time T, initial gravity and poses)
      • Lines 3–6: Sensor data acquisition in loop
      • Lines 7–10: IMU alignment/calibration phase
      • Lines 12–16: IMU+DVL fusion and pose update
    """
    dvl_vel_data = data.dvl_velocities  # νD readings
    imu_ang_vel_data = data.imu_angular_velocities  # ωI readings
    imu_lin_acc_data = data.imu_linear_accelerations  # gI static component

    # Known DVL-to-body rotation (r_B_D = 180° about x-axis)
    r_B_D = dq.i_

    # Initialize pose and twist
    x_W_B_kminus1 = initial_pos  # x̂W_B[0]
    T = 1 / freq               # Sampling time T (line 1)
    # twist_W_Bkminus1_wrt_W = 0  # ξ̂W_W,B[0]

    end = len(dvl_vel_data[0])
    # g_error_vec = np.zeros((3, end + 1))

    # Calibration: estimate IMU-to-body misalignment (Algorithm 1 lines 7–10)
    g_avg = dq.DQ([0])          # ḡI[0]
    for k in range(0, calibration_time + 1):
        g_I_k = [imu_lin_acc_data[0, k], imu_lin_acc_data[1, k], imu_lin_acc_data[2, k]]
        r_hat_B_I_k, g_avg = rotation_estimate_step(g_I_k, k, g_avg, r_hat_B_I_kminus1, T)
        r_hat_B_I_kminus1 = r_hat_B_I_k
    # Prepare outputs for dead reckoning path
    DR_x_and_y = np.zeros((2, (end - calibration_time + 1)))
    start_pt = dq.vec3(initial_pos.translation())
    DR_x_and_y[:, 0] = start_pt[:2]
    yaw = np.zeros(end - calibration_time + 1)
    yaw[0] = x_W_B_kminus1.rotation_angle()
    index_for_DR = 1

    # Dead reckoning loop (Algorithm 1)
    for k in range(calibration_time + 1, end):
        # pull single occurences of data from stored array
        g_I_k = [imu_lin_acc_data[0, k], imu_lin_acc_data[1, k], imu_lin_acc_data[2, k]]
        w_I = [imu_ang_vel_data[0, k], imu_ang_vel_data[1, k], imu_ang_vel_data[2, k]]
        v_D_k = [dvl_vel_data[0, k], dvl_vel_data[1, k], dvl_vel_data[2, k]]

        r_hat_B_I_k, g_avg = rotation_estimate_step(g_I_k, k, g_avg, r_hat_B_I_kminus1, T)
        
        x_W_B_k = dead_reckoning_step(r_hat_B_I_k, r_B_D, w_I, v_D_k, x_W_B_kminus1, T)

        r_hat_B_I_kminus1 = r_hat_B_I_k
        x_W_B_kminus1 = x_W_B_k

        # Record dead-reckoned position
        pt = dq.vec3(x_W_B_k.translation())
        DR_x_and_y[:, index_for_DR] = pt[:2]
        yaw[index_for_DR] = x_W_B_k.rotation_angle()
        index_for_DR += 1

    return DR_x_and_y
