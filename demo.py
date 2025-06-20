#!/usr/bin/env python

import algorithm as dr
import numpy as np
from numpy.testing import assert_almost_equal, assert_equal
from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection
import dqrobotics as dq
from data_loader import LoadExperimentData
from algorithm import DeadReckoning


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
    data = LoadExperimentData(experiment_number)
 
    dr = DeadReckoning()
    deadreckon = dr.generate_dualQ(data)


    print(deadreckon.shape)
    print(deadreckon [:,8670])



    plot_traj(
        data.time_stamps,
        data.calibration_time,
        data.vicon_positions,
        deadreckon,
        # experiment_name=experiment_name,
    )


if __name__ == "__main__":
    # your experiment here:
    experiment_number = 7
    # sanity check
    assert 1 <= experiment_number <= 10, "experiment_number must be 1–10"

    main(experiment_number)


#### Test ######    
# In default mode this code should output:
# > (2, 8672)
# > [0.72854136 0.83409027]

