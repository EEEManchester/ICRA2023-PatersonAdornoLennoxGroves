#!/usr/bin/env python3

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


class DeadStep:
    def __init__(self):
        self.gain = 10
        self.g_B = dq.k_
        freq = 50
        self.T = 1/freq
        self.r_B_D = dq.i_
        # Initialize imu gravity vector rolling average
        self.g_avg_I = dq.DQ([0])
    

    def rotation_estimator(self, k, g_I_k, r_hat_B_I_kminus1):
        ######## IMU alignment/calibration phase (Algorithm 1 lines 7–10): ######
        # Normalize current IMU gravity vector measurement
        g_I_DQ = dq.DQ(g_I_k)
        g_I_normalised = dq.normalize(g_I_DQ)
        # Update zero-frequency (running average) gravity vector in IMU frame (line 7)
        self.g_avg_I = dq.DQ(dq.vec3(g_I_normalised) / (k + 1)) \
                  + (k / (k + 1)) * self.g_avg_I
        # Estimate gravity in body frame using current rotation (line 8)
        g_hat_B_k = dq.Ad(r_hat_B_I_kminus1, self.g_avg_I)
        # Compute correction angular velocity with gain λ=10 (line 9)
        w_hat_B_BI_k = self.gain * dq.cross(g_hat_B_k, self.g_B)
        # Update IMU-to-body rotation via exponential map (line 10)
        r_hat_B_I_k = dq.exp(0.5 * self.T * w_hat_B_BI_k) * r_hat_B_I_kminus1
        # move to next step

        return r_hat_B_I_k

    def velocity_pose_update(self, r_hat_B_I_k, w_I, v_D_k, x_W_B_kminus1):
        ######## IMU+DVL fusion and pose update (Algorithm 1 lines 12–16):#######
        # Body-frame angular velocity ω̂W,B (line 12)
        w_hat_B_WB = dq.Ad(r_hat_B_I_k, dq.DQ(w_I))
        # DVL linear velocity projection (line 13)
        p_hat_dot_B_WB = dq.Ad(self.r_B_D, dq.DQ(v_D_k))
        # Combine angular + linear into dual twist ξ̂B_W,B (line 15)
        twist_hat_B_WB = w_hat_B_WB + dq.E_ * p_hat_dot_B_WB
        # Integrate pose with world-frame twist (line 16)
        x_W_B_k = x_W_B_kminus1 * dq.exp((self.T / 2) * twist_hat_B_WB)
        # move to next step
        # x_W_B_kminus1 = x_W_B_k    -------- removed

        return x_W_B_k
    
    # def combine_rotation_pose (self, k, g_I_k, w_I, v_D_k, r_hat_B_I_kminus1, x_W_B_kminus1, calibration_time):
        
    #     r_hat_B_I_k = rotation_estimator (self, k, g_I_k, r_hat_B_I_kminus1)
    #     if k > calibration_time:
    #         x_W_B_k = velocity_pose_update (self, r_hat_B_I_k, w_I, v_D_k, x_W_B_kminus1)
    #         return x_W_B_k
    #     return r_hat_B_I_k


 


