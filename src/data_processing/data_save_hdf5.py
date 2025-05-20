import argparse
import json
import os

import cv2
import h5py
import numpy as np
import pandas as pd
from tqdm import tqdm

# Load configuration from config.json
with open("config/config.json") as f:
    config = json.load(f)

TASK_CONFIG = config["task_config"]
cfg = TASK_CONFIG

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="grasp_cube_v1_22")
parser.add_argument("--num_episodes", type=int, default=6)
args = parser.parse_args()
task = args.task
num_episodes = args.num_episodes

# Data path
data_path = os.path.join(config["device_settings"]["data_dir"], "dataset", str(task))

IMAGE_PATH = os.path.join(data_path, "camera/")
CSV_PATH = os.path.join(data_path, "csv/")
VIDEO_PATH_TEMP = os.path.join(data_path, "camera", "temp_video_n.mp4")
TRAJECTORY_PATH_TEMP = os.path.join(data_path, "csv", "temp_trajectory_n.csv")
TIMESTAMP_PATH_TEMP = os.path.join(data_path, "csv", "temp_video_timestamps_n.csv")

for episode in range(num_episodes):
    # Data list preparation
    data_dict = {
        "/observations/qpos": [],
        "/action": [],
    }
    for cam_name in cfg["camera_names"]:
        data_dict[f"/observations/images/{cam_name}"] = []

    # Process video frames
    timestamps = pd.read_csv(TIMESTAMP_PATH_TEMP.replace("_n", f"_{episode}"))
    downsampled_timestamps = timestamps.iloc[::3].reset_index(drop=True)
    cap = cv2.VideoCapture(VIDEO_PATH_TEMP.replace("_n", f"_{episode}"))

    for idx, row in tqdm(downsampled_timestamps.iterrows(), desc="Extracting Images"):
        frame_idx = row["Frame Index"]
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()
        if ret:
            for cam_name in cfg["camera_names"]:
                # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                data_dict[f"/observations/images/{cam_name}"].append(frame)

    cap.release()

    # Process trajectory data
    trajectory = pd.read_csv(TRAJECTORY_PATH_TEMP.replace("_n", f"_{episode}"))
    trajectory["Timestamp"] = trajectory["Timestamp"].astype(float)

    for idx, row in tqdm(downsampled_timestamps.iterrows(), desc="Extracting States"):
        closest_idx = (np.abs(trajectory["Timestamp"] - row["Timestamp"])).argmin()
        closest_row = trajectory.iloc[closest_idx]
        pos_quat = [
            closest_row["Pos X"],
            closest_row["Pos Y"],
            closest_row["Pos Z"],
            closest_row["Q_X"],
            closest_row["Q_Y"],
            closest_row["Q_Z"],
            closest_row["Q_W"],
        ]
        data_dict["/observations/qpos"].append(pos_quat)
        data_dict["/action"].append(pos_quat)

    dataset_path = os.path.join(data_path, f"episode_{episode}.hdf5")
    os.makedirs(os.path.dirname(dataset_path), exist_ok=True)

    # Save the data
    with h5py.File(dataset_path, "w", rdcc_nbytes=2 * 1024**2) as root:
        root.attrs["sim"] = False
        obs = root.create_group("observations")
        image_grp = obs.create_group("images")
        for cam_name in cfg["camera_names"]:
            image_grp.create_dataset(
                cam_name,
                data=np.array(data_dict[f"/observations/images/{cam_name}"], dtype=np.uint8),
                compression="gzip",
                compression_opts=4,
            )
        root.create_dataset("observations/qpos", data=np.array(data_dict["/observations/qpos"]))
        root.create_dataset("action", data=np.array(data_dict["/action"]))

    print(f"episode_{episode} done!")

print("All episodes completed successfully!")
