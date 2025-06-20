#!/usr/bin/env python

import pandas as pd

class LoadExperimentData:
    def __init__(self, experiment_number: int):
        self.csv_dir = "preprocessed/"
        self.experiments = {
            1: "03_xdata",
            2: "04_ydata",
            3: "05_yawdata",
            4: "06_squaresfixedyawdata",
            5: "07_DVLxydata",
            6: "08_fig8fixedyawdata",
            7: "09_fig8DVLxydata",
            8: "10_fig8DVLxydata",
            9: "11_rnd_drive_data",
            10: "12_all",
        }

        self.experiment_name = self.experiments[experiment_number]

        self.vicon_path = f"{self.csv_dir}{self.experiment_name}_vicon.csv"
        self.imu_path = f"{self.csv_dir}{self.experiment_name}_imu.csv"
        self.dvl_path = f"{self.csv_dir}{self.experiment_name}_dvl.csv"

        self.loaded_csvs = self._load_csvs()
        self._extract_data()

    def _load_csvs(self):
        return {
            "vicon_data": pd.read_csv(self.vicon_path),
            "imu_data": pd.read_csv(self.imu_path),
            "dvl_data": pd.read_csv(self.dvl_path),
        }

    def _extract_data(self):
        self.time_stamps = self.loaded_csvs["vicon_data"]["time"].to_numpy()

        self.dvl_velocities = self.loaded_csvs["dvl_data"][
            ["velocity.x", "velocity.y", "velocity.z"]
        ].to_numpy().T

        self.imu_angular_velocities = self.loaded_csvs["imu_data"][
            [
                "angular_velocities.x",
                "angular_velocities.y",
                "angular_velocities.z",
            ]
        ].to_numpy().T

        self.imu_linear_accelerations = self.loaded_csvs["imu_data"][
            [
                "linear_acceleration.x",
                "linear_acceleration.y",
                "linear_acceleration.z",
            ]
        ].to_numpy().T

        self.vicon_positions = self.loaded_csvs["vicon_data"][
            ["translation.x", "translation.y", "translation.z"]
        ].to_numpy().T

        self.vicon_orientation = self.loaded_csvs["vicon_data"][
            ["rotation.w", "rotation.x", "rotation.y", "rotation.z"]
        ].to_numpy().T
