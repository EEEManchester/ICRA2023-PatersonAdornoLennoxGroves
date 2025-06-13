#!/usr/bin/env python

"""
This file is a python port of an identically named Matlab version.
The following comment is reproduced directly from there:
Simple script to evaluate the data fusion algorithm from the IMU and DVL.
It is a result from weekly meetings between Jessica Paterson, Keir
Groves, and Bruno Adorno

Conventions:
 - VARIABLES that would normally be written as $x^{sup}_{sub}$
in LaTeX, such as poses and rotations, are written as x_sup_sub in the
code. That is, the first string after the first underscore, sup, is the
superscript whereas the second string after the second underscore, sub,
is the subscript.

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
    Process a single gravity vector measurement to update the IMU rotation estimate.
    """
    # g_I_k: A list or array of 3 values — the gravity vector measured by the IMU at time step k, in the IMU frame.
    # k: The current time step index. Used for computing the running average of gravity vectors.
    # g_avg_previous: The previously computed average of gravity vectors. Used to update the average incrementally.
    # r_W_I_estimated: The current estimate of the rotation from the IMU frame to the world frame, represented as a dual quaternion.
    # T: The time step duration (sampling period), used in the exponential map for updating the rotation.


    # Make current IMU frame gravity vector a DQ object and normalise it
    g_I_DQ = dq.DQ(g_I_k)
    g_I_normalised = dq.normalize(g_I_DQ)

    # Algorithm line 7 in paper
    # Running average of gravity vector in imu frame - slightly different from paper to avoid division by 0
    g_avg = dq.DQ(dq.vec3(g_I_normalised) / (k + 1)) + (k / (k + 1)) * g_avg_previous

    # Algrithm line 8 n paper
    # Estimate gravity in world frame
    g_B_estimated = dq.Ad(r_B_I_estimated, g_avg)

    # Algorithm line 9 in paper
    # Compute correction anglular velocity. Lambda(gain)=10 as per the paper
    gain = 10
    w_B_B_I = gain * dq.cross(g_B_estimated, dq.k_)

    # Algorithm line 10 in paper
    # update the rotation of
    r_B_I_estimated = dq.exp(0.5 * dt * w_B_B_I) * r_B_I_estimated

    # Compute error
    g_error = g_B_estimated - dq.k_
    norm_g_error = dq.vec8(dq.norm(g_error))[0]

    return r_B_I_estimated, norm_g_error, g_avg


def imu_rotation_generator(g_I, r_W_I_estimated, dt):
    """
    Generator that yields updated IMU rotation estimates over time.
    """
    r_W_I_estimated = dq.normalize(r_W_I_estimated)
    g_avg_previous = dq.DQ([0])

    for k in range(g_I.shape[1]):
        g_I_k = [g_I[0, k], g_I[1, k], g_I[2, k]]
        r_W_I_estimated, norm_g_error, g_avg = process_single_gravity_measurement(
            g_I_k, k, g_avg_previous, r_W_I_estimated, dt
        )
        g_avg_previous = g_avg
        yield r_W_I_estimated, norm_g_error, g_avg

def generate_dualQ(
    data,
    calibration_time,
    r_W_I_estimated=dq.DQ([1]),
    freq=50,
    initial_pos=dq.DQ([1, 0, 0, 0, 0, 0, 0, 0]),
):
    dvl_vel_data = data.dvl_velocities
    imu_ang_vel_data = data.imu_angular_velocities
    imu_lin_acc_data = data.imu_linear_accelerations

    # DVL rotation
    r_B_D = dq.i_

    # Pose of the body frame at instant k-1 with respect to the world
    #  (they start aligned)
    x_W_Bkminus1 = initial_pos
    # Sampling period used in the numerical integration in seconds
    dt = 1 / freq

    # Initial twist between the body frame at instant k-1 and the w.r.t. world.
    twist_W_Bkminus1_wrt_W = 0

    end = len(dvl_vel_data[0]) 

    norm_g_error = np.zeros(end)

    imu_rotation = imu_rotation_generator(g_I=imu_lin_acc_data, r_W_I_estimated=r_W_I_estimated, dt=dt)

    w_Bkminus1_Bk_wrt_Bkminus1 = dq.DQ([1])
    w_Bkminus1_Bk_wrt_Bkminus1_vec = np.zeros(end)
    g_vec = np.zeros((3,end +1))
    # Calibrate the IMU rotation
    for k in range(0, calibration_time + 1):
        r_W_I_estimated, norm_g_error[k], gavg = next(imu_rotation)
        g_vec[:,k+1] = dq.vec3(gavg)
        w_Ikminus1_Ik_wrt_Ikminus1 = dq.DQ(
            [imu_ang_vel_data[0, k], imu_ang_vel_data[1, k], imu_ang_vel_data[2, k]]
        )

        w_Bkminus1_Bk_wrt_Bkminus1 = dq.Ad(r_W_I_estimated, w_Ikminus1_Ik_wrt_Ikminus1)
        w_Bkminus1_Bk_wrt_Bkminus1_vec[k] = dq.vec3(w_Bkminus1_Bk_wrt_Bkminus1)[2]

    DR_x_and_y = np.zeros((2, (end - calibration_time + 1)))
    # Store initial point
    start_point = dq.vec3(initial_pos.translation())
    index_for_DR = 0
    DR_x_and_y[0, index_for_DR] = start_point[0]
    DR_x_and_y[1, index_for_DR] = start_point[1]
    yaw =  np.zeros(end - calibration_time +1)
    yaw[index_for_DR] = x_W_Bkminus1.rotation_angle()

    for k in range(calibration_time + 1, end):
        r_W_I_estimated, norm_g_error[k], gavg = next(imu_rotation)
        g_vec[:,k+1] = dq.vec3(gavg)

        # Measured angular velocity in the IMU/BODY/DVL frame. Because of the
        # discretization, this angular velocity is the velocity of the body
        # frame at instant k (B_k) with respect to the body frame at
        # k-1 (B_{k-1}),  expressed in the body frame at k-1 (B_{k-1})

        w_Ikminus1_Ik_wrt_Ikminus1 = dq.DQ(
            [imu_ang_vel_data[0, k], imu_ang_vel_data[1, k], imu_ang_vel_data[2, k]]
        )

        w_Bkminus1_Bk_wrt_Bkminus1 = dq.Ad(r_W_I_estimated, w_Ikminus1_Ik_wrt_Ikminus1)
        w_Bkminus1_Bk_wrt_Bkminus1_vec[k] = dq.vec3(w_Bkminus1_Bk_wrt_Bkminus1)[2]

        # Measured linear velocity (v) in the IMU/BODY/DVL frame.
        # The explanation is the same as for the angular velocity.
        # v_Dkminus1_Dk_wrt_Dkminus1 = (velocities[0][i] * dq.i_ + velocities[1][i] * dq.j_ + velocities[2][i] * dq.k_)
        v_Dkminus1_Dk_wrt_Dkminus1 = dq.DQ(
            [dvl_vel_data[0, k], dvl_vel_data[1, k], dvl_vel_data[2, k]]
        )

        v_Bkminus1_Bk_wrt_Bkminus1 = dq.Ad(r_B_D, v_Dkminus1_Dk_wrt_Dkminus1)

        # The measured twist in the body frame is just the composition of the
        # angular and linear velocities
        twist_Bkminus1_Bk_wrt_Bkminus1 = (
            w_Bkminus1_Bk_wrt_Bkminus1 + dq.E_ * v_Bkminus1_Bk_wrt_Bkminus1
        )

        # The measured body twist between two consecutive frames, expressed in
        # the world frame, is found by means of the adjoint operation. This is
        # needed because we need to integrate the pose for dead-reckoning.
        # So, all quantities are expressed in the world frame.
        twist_Bkminus1_Bk_wrt_W = dq.Ad(x_W_Bkminus1, twist_Bkminus1_Bk_wrt_Bkminus1)

        # The updated twist between the body frame and the world frame at instant k
        # is the sum of the twist between the body frames B_{k-1} and B_{k} (i.e.,
        # the update) with the twist between the world frame and the body frame at
        # instant k-1.
        twist_W_Bk_wrt_W = twist_W_Bkminus1_wrt_W + twist_Bkminus1_Bk_wrt_W

        # Numerically integrate the pose using the up-to-date measured twist in the
        # world frame
        x_W_Bk = dq.exp((dt / 2) * twist_W_Bk_wrt_W) * x_W_Bkminus1
        x_W_Bkminus1 = x_W_Bk

        dual_quat_translation = dq.vec3(x_W_Bk.translation())
        yaw[index_for_DR] = x_W_Bk.rotation_angle()

        index_for_DR += 1
        DR_x_and_y[0, index_for_DR] = dual_quat_translation[0]
        DR_x_and_y[1, index_for_DR] = dual_quat_translation[1]

    return (DR_x_and_y)