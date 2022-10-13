#!/usr/bin/env python

import argparse
import sys

import algorithm as dr
import pandas as pd
import numpy as np
from numpy.testing import assert_almost_equal, assert_equal
from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection
import dqrobotics as dq


class CSVDataLoader:
    def __init__(
        self,
        vicon_csv_path: str,
        imu_csv_path: str,
        dvl_csv_path: str,
    ):
        self.csvs_to_load = {
            "vicon_data": (vicon_csv_path),
            "imu_data": (imu_csv_path),
            "dvl_data": (dvl_csv_path),
        }

    def get_csvs(self):
        return {
            csv: pd.read_csv(file_name)
            for (csv, file_name) in self.csvs_to_load.items()
        }


class Data:
    def __init__(self, csv_loader) -> None:
        self.loaded_csvs = csv_loader.get_csvs()
        self.time_stamps = self.loaded_csvs["vicon_data"]["time"].to_numpy()
        self.dvl_velocities = (
            self.loaded_csvs["dvl_data"][
                ["velocity.x", "velocity.y", "velocity.z"]
            ]
            .to_numpy()
            .T
        )
        self.imu_angular_velocities = (
            self.loaded_csvs["imu_data"][
                [
                    "angular_velocities.x",
                    "angular_velocities.y",
                    "angular_velocities.z",
                ]
            ]
            .to_numpy()
            .T
        )
        self.imu_linear_accelerations = (
            self.loaded_csvs["imu_data"][
                [
                    "linear_acceleration.x",
                    "linear_acceleration.y",
                    "linear_acceleration.z",
                ]
            ]
            .to_numpy()
            .T
        )
        self.vicon_positions = (
            self.loaded_csvs["vicon_data"][
                ["translation.x", "translation.y", "translation.z"]
            ]
            .to_numpy()
            .T
        )
        self.vicon_orientation = (
            self.loaded_csvs["vicon_data"][
                ["rotation.w", "rotation.x", "rotation.y", "rotation.z"]
            ]
            .to_numpy()
            .T
        )


def plot_traj(
    time_stamps,
    calibration_time,
    ground_truth,
    dead_reckoning,
    experiment_name=None,
    seconds_to_plot=120,
):
    fig, ax = plt.subplots(figsize=(13, 7))

    # Control how much of the experiment is plotted
    (seconds_to_plot_index,) = np.where(time_stamps == seconds_to_plot)
    first_point_ground_truth = calibration_time

    # Plotting the whole experiment is possible by setting the last points to -1
    last_point_ground_truth = seconds_to_plot_index[0] + calibration_time
    last_point_dr = seconds_to_plot_index[0]

    gt_x = ground_truth[0, first_point_ground_truth:last_point_ground_truth]
    gt_y = ground_truth[1, first_point_ground_truth:last_point_ground_truth]
    dr_x = dead_reckoning[0, 0:last_point_dr]
    dr_y = dead_reckoning[1, 0:last_point_dr]

    # Check first points are equal
    assert_almost_equal(gt_x[0], dr_x[0], decimal=15)
    assert_almost_equal(gt_y[0], dr_y[0], decimal=15)

    # Check plotting same number of data points
    assert_equal(
        ground_truth[0, first_point_ground_truth:last_point_ground_truth].size,
        dr_x.size,
    )
    assert_equal(
        ground_truth[1, first_point_ground_truth:last_point_ground_truth].size,
        dr_y.size,
    )

    points = np.array([dr_x, dr_y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    lc = LineCollection(
        segments, cmap="viridis", linewidth=2, label="Dead Reckoning", zorder=1
    )
    # set color to date values
    lc.set_array(time_stamps[first_point_ground_truth:last_point_ground_truth])
    line = ax.add_collection(lc)

    points_gt = np.array([gt_x, gt_y]).T.reshape(-1, 1, 2)
    segments_gt = np.concatenate([points_gt[:-1], points_gt[1:]], axis=1)

    lc_gt = LineCollection(
        segments_gt,
        linestyles="--",
        cmap="viridis",
        linewidth=2,
        label="Ground Truth",
    )
    # set color to date values
    lc_gt.set_array(
        time_stamps[first_point_ground_truth:last_point_ground_truth]
    )

    ax.add_collection(lc_gt)
    fig.colorbar(line, label="Time (s)", pad=0.01)

    ax.plot(
        ground_truth[0, first_point_ground_truth:last_point_ground_truth],
        ground_truth[1, first_point_ground_truth:last_point_ground_truth],
        ":",
        color="white",
        zorder=2,
    )

    ax.plot(
        gt_x[0],
        gt_y[0],
        marker="o",
        color="white",
        markerfacecolor="None",
        markersize=15,
        markeredgecolor="black",
        markeredgewidth=2,
        label="Start",
    )
    # Mark end points
    ax.plot(
        ground_truth[0, last_point_ground_truth - 1],
        ground_truth[1, last_point_ground_truth - 1],
        marker="x",
        markersize=15,
        color="white",
        markeredgecolor="black",
        markeredgewidth=2,
        label="End Points",
    )

    ax.plot(
        dead_reckoning[0, last_point_dr - 1],
        dead_reckoning[1, last_point_dr - 1],
        marker="x",
        markersize=15,
        markeredgecolor="black",
        markeredgewidth=2,
    )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")

    ax.legend()
    ax.set_ylim([0.0, 2.6])

    ax.xaxis.grid()
    ax.yaxis.grid()
    if experiment_name:
        ax.set_title(experiment_name)
    plt.show()


def main(experiment_number):
    experiments = {
        1: "03_xdata",  #
        2: "04_ydata",
        3: "05_yawdata",
        4: "06_squaresfixedyawdata",
        5: "07_DVLxydata",
        6: "08_fig8fixedyawdata",
        7: "09_fig8DVLxydata",  #
        8: "10_fig8DVLxydata",
        9: "11_rnd_drive_data",
        10: "12_all",
    }

    # Load csvs
    # This assumes all file names follow the pattern <experiment_name>_<sensor_name>.csv
    # Where sensor name is vicon, imu or dvl
    csv_dir = "preprocessed/"
    experiment_name = experiments[experiment_number]

    vicon_data = csv_dir + experiment_name + "_vicon.csv"
    imu_data = csv_dir + experiment_name + "_imu.csv"
    dvl_data = csv_dir + experiment_name + "_dvl.csv"
    data = Data(
        CSVDataLoader(
            imu_csv_path=imu_data,
            vicon_csv_path=vicon_data,
            dvl_csv_path=dvl_data,
        )
    )

    ground_truth = data.vicon_positions

    vicon_orientation_quat = data.vicon_orientation

    time_stamps = data.time_stamps
    (seconds_to_calibrate,) = np.where(time_stamps == 4)
    calibration_time = seconds_to_calibrate[0]

    vicon_initial_pos = (
        ground_truth[0, calibration_time] * dq.i_
        + ground_truth[1, calibration_time] * dq.j_
        + ground_truth[2, calibration_time] * dq.k_
    )

    initial_orientation = (
        vicon_orientation_quat[0, calibration_time]
        + vicon_orientation_quat[1, calibration_time] * dq.i_
        + vicon_orientation_quat[2, calibration_time] * dq.j_
        + vicon_orientation_quat[3, calibration_time] * dq.k_
    )

    initial_pos = (1 + (dq.E_ * vicon_initial_pos) * 0.5) * dq.normalize(
        initial_orientation
    )

    # Initial guess of IMU rotation based on observations recorded during experiment
    r_W_I_estimated = (np.cos(-np.pi / 4) + dq.k_ * np.sin(-np.pi / 4)) * (
        np.cos(np.pi / 4) + dq.i_ * np.sin(np.pi / 4)
    )

    deadreckon = dr.generate_dualQ(
        data,
        calibration_time=calibration_time,
        r_W_I_estimated=r_W_I_estimated,
        initial_pos=initial_pos,
    )

    plot_traj(
        time_stamps,
        calibration_time,
        ground_truth,
        deadreckon,
        experiment_name=experiment_name,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Demo.py Plot dead reckoning trajectories"
    )

    parser.add_argument(
        "experiment_number",
        metavar="exp_no",
        type=int,
        choices=range(1, 11),
        help="An integer in the range 1-10 inclusive",
    )

    if len(sys.argv) == 1:
        parser.print_help()
        parser.exit()

    args = parser.parse_args()
    main(args.experiment_number)
