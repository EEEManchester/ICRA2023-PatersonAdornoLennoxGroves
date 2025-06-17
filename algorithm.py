#!/usr/bin/env python

"""
This file is a Python port of an identically named MATLAB version.
Simple script to evaluate the data fusion algorithm from the IMU and DVL.
Result of weekly meetings between Jessica Paterson, Keir Groves, and Bruno Adorno.

Conventions:
 - Variables x_sup_sub represent x^{sup}_{sub} in LaTeX
 - Variables p_sub1_sub2_wrt_sup represent p^{sup}_{sub1,sub2} in LaTeX
 --> FOR EXAMPLE, x_1_2, which in LaTeX would be $x^1_2$ might represent
the rigid motion from the initial frame F_1 to the final frame F_2.

- VARIABLES that would normally be written as $p^{sup}_{sub1,sub2}$ in
LaTex, such as translations and twists, are written as
p_sub1_sub2_wrt_sup in the code.

--> FOR EXAMPLE, p_1_2_wrt_1, which in LaTeX would be written as
$p^1_{1,2}$ might represent the translation from the initial frame F_1 to
the final frame F_2 with respect to the frame F_1 (i.e., the coordinates
of p are expressed in frame F_1).
--> FOR EXAMPLE, twist_WORLD_IMU_wrt_WORLD might represent the twist
between the WORLD and the IMU with respect to the WORLD (that is, the
action that generates the twist is seen from the world frame).
--> FOR EXAMPLE, twist_WORLD_IMU_wrt_ANOTHERFRAME might represent the twist
between the WORLD and the IMU with respect to ANOTHERFRAME (that is, the
action that generates the twist is seen from ANOTHERFRAME, which might be
neither the world frame nor the IMU frame.


Current assumptions for this simulation:
  1) The IMU, DVL, and body frames are aligned and located at the CoM.
  2) The world frame is defined as the body frame when k=0.

Warning about notation: We are using the discrete time k. Not to confuse
with the imaginary unit k_ !
"""
import numpy as np
import dqrobotics as dq


def process_single_gravity_measurement(g_I_k, k, g_avg_previous, r_B_I_estimated, dt):
    """
    Process one gravity vector measurement to update IMU-to-body rotation estimate.
    Corresponds to Algorithm 1 lines 7–10 in the paper:
      7. Compute running average of gravity: ḡI[n] = (1/n) gI[n] + ((n−1)/n) ḡI[n−1]
      8. Estimate gravity in body frame: ĝB[n] = Ad(r̂B_I[n−1]) ḡI[n]
      9. Compute correction angular velocity: ω̂B_B,I[n] = λ (ĝB[n] × gB)
     10. Update rotation: r̂B_I[n] = exp((T/2) ω̂B_B,I[n]) r̂B_I[n−1]
    """
    # Normalize current IMU gravity vector measurement (Algorithm 1 line 7)
    g_I_DQ = dq.DQ(g_I_k)
    g_I_normalised = dq.normalize(g_I_DQ)

    # Update zero-frequency (running average) gravity in IMU frame (line 7)
    g_avg = dq.DQ(dq.vec3(g_I_normalised) / (k + 1)) \
            + (k / (k + 1)) * g_avg_previous

    # Estimate gravity in body frame using current rotation (line 8)
    g_B_estimated = dq.Ad(r_B_I_estimated, g_avg)

    # Compute correction angular velocity with gain λ=10 (line 9)
    gain = 10
    w_B_B_I = gain * dq.cross(g_B_estimated, dq.k_)

    # Update IMU-to-body rotation via exponential map (line 10)
    r_B_I_estimated = dq.exp(0.5 * dt * w_B_B_I) * r_B_I_estimated

    # Compute and return gravity error norm
    g_error = g_B_estimated - dq.k_
    norm_g_error = dq.vec8(dq.norm(g_error))[0]

    return r_B_I_estimated, norm_g_error, g_avg


def generate_dualQ(
    data,
    calibration_time,
    r_B_I_estimated=dq.DQ([1]),
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
    x_W_Bkminus1 = initial_pos  # x̂W_B[0]
    dt = 1 / freq               # Sampling time T (line 1)
    # twist_W_Bkminus1_wrt_W = 0  # ξ̂W_W,B[0]

    end = len(dvl_vel_data[0])
    # g_error_vec = np.zeros((3, end + 1))

    # Calibration: estimate IMU-to-body misalignment (Algorithm 1 lines 7–10)
    g_avg = dq.DQ([0])          # ḡI[0]
    for k in range(0, calibration_time + 1):
        g_I_k = [imu_lin_acc_data[0, k], imu_lin_acc_data[1, k], imu_lin_acc_data[2, k]]
        r_B_I_estimated, norm_g_error, g_avg = \
            process_single_gravity_measurement(g_I_k, k, g_avg, r_B_I_estimated, dt)

    # Prepare outputs for dead reckoning path
    DR_x_and_y = np.zeros((2, (end - calibration_time + 1)))
    start_pt = dq.vec3(initial_pos.translation())
    DR_x_and_y[:, 0] = start_pt[:2]
    yaw = np.zeros(end - calibration_time + 1)
    yaw[0] = x_W_Bkminus1.rotation_angle()
    index_for_DR = 1

    # Dead reckoning loop (Algorithm 1 lines 12–16)
    for k in range(calibration_time + 1, end):
        # Update rotation estimate from gravity (repeats lines 7–10)
        g_I_k = [imu_lin_acc_data[0, k], imu_lin_acc_data[1, k], imu_lin_acc_data[2, k]]
        r_B_I_estimated, norm_g_error, g_avg = \
            process_single_gravity_measurement(g_I_k, k, g_avg, r_B_I_estimated, dt)

        # Body-frame angular velocity ω̂W,B[n] (line 12)
        w_Bkminus1_Bk_wrt_Bkminus1 = dq.Ad(r_B_I_estimated,
            dq.DQ([imu_ang_vel_data[0, k], imu_ang_vel_data[1, k], imu_ang_vel_data[2, k]]))

        # DVL linear velocity projection (line 13)
        v_B = dq.Ad(r_B_D,
            dq.DQ([dvl_vel_data[0, k], dvl_vel_data[1, k], dvl_vel_data[2, k]]))

        # Combine angular + linear into dual twist ξ̂B_W,B[n] (line 15)
        twist = w_Bkminus1_Bk_wrt_Bkminus1 + dq.E_ * v_B

        # Integrate pose with world-frame twist (line 16)
        x_W_Bk = x_W_Bkminus1 * dq.exp((dt / 2) * twist)
        x_W_Bkminus1 = x_W_Bk

        # Record dead-reckoned position
        pt = dq.vec3(x_W_Bk.translation())
        DR_x_and_y[:, index_for_DR] = pt[:2]
        yaw[index_for_DR] = x_W_Bk.rotation_angle()
        index_for_DR += 1

    return DR_x_and_y
