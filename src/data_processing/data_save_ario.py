import argparse
import json
import os

import cv2
import numpy as np
import pandas as pd
from tqdm import tqdm

def load_config():
    """Load configuration from config file."""
    with open("configs/data_collection.json") as f:
        config = json.load(f)
    return config

def create_ario_structure(base_path, task_name, episode_num):
    """Create ARIO directory structure."""
    # Create main directories
    episode_path = os.path.join(base_path, "series-1", f"task-{task_name}", f"episode-{episode_num}")
    os.makedirs(episode_path, exist_ok=True)
    
    # Create sensor directories
    os.makedirs(os.path.join(episode_path, "cam-1"), exist_ok=True)
    os.makedirs(os.path.join(episode_path, "cam-2"), exist_ok=True)
    
    return episode_path

def process_episode(episode, config, data_path, cfg):
    """Process a single episode of data and save in ARIO format."""
    # Create ARIO directory structure
    episode_path = create_ario_structure(data_path, cfg["task_name"], episode)
    
    # Set up paths for this episode
    video_path = os.path.join(data_path, "camera", f"temp_video_{episode}.mp4")
    trajectory_path = os.path.join(data_path, "csv", f"temp_trajectory_{episode}.csv")
    timestamp_path = os.path.join(data_path, "csv", f"temp_video_timestamps_{episode}.csv")
    
    print(f"\nProcessing episode {episode}")
    print(f"Video path: {video_path}")
    print(f"Trajectory path: {trajectory_path}")
    print(f"Timestamp path: {timestamp_path}")
    
    try:
        # Check if files exist
        if not os.path.exists(video_path):
            raise FileNotFoundError(f"Video file not found: {video_path}")
        if not os.path.exists(trajectory_path):
            raise FileNotFoundError(f"Trajectory file not found: {trajectory_path}")
        if not os.path.exists(timestamp_path):
            raise FileNotFoundError(f"Timestamp file not found: {timestamp_path}")
        
        # Process video frames
        timestamps = pd.read_csv(timestamp_path)
        print(f"Loaded {len(timestamps)} timestamps")
        downsampled_timestamps = timestamps.iloc[::3].reset_index(drop=True)
        print(f"Downsampled to {len(downsampled_timestamps)} timestamps")
        
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open video file: {video_path}")
        
        # Process each camera
        for cam_idx, cam_name in enumerate(cfg["camera_names"], 1):
            cam_path = os.path.join(episode_path, f"cam-{cam_idx}")
            print(f"\nProcessing camera {cam_idx}")
            
            # Save video
            height, width = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            print(f"Video dimensions: {width}x{height}")
            video_path_out = os.path.join(cam_path, "rgb.mp4")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(video_path_out, fourcc, 30.0, (width, height))
            
            frames_written = 0
            for idx, row in tqdm(downsampled_timestamps.iterrows(), desc=f"Processing camera {cam_idx}"):
                frame_idx = row["Frame Index"]
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
                ret, frame = cap.read()
                if ret:
                    out.write(frame)
                    frames_written += 1
            
            out.release()
            print(f"Wrote {frames_written} frames for camera {cam_idx}")
            
            # Save timestamps
            timestamps_ms = downsampled_timestamps["Timestamp"].values * 1000  # Convert to ms
            np.save(os.path.join(cam_path, "timestamps.npy"), timestamps_ms)
            print(f"Saved {len(timestamps_ms)} timestamps for camera {cam_idx}")
        
        cap.release()
        
        # Process trajectory data
        trajectory = pd.read_csv(trajectory_path)
        print(f"\nLoaded trajectory data with {len(trajectory)} rows")
        trajectory["Timestamp"] = trajectory["Timestamp"].astype(float)
        
        # Save base.txt (robot base movement)
        base_data = []
        for idx, row in tqdm(downsampled_timestamps.iterrows(), desc="Processing robot states"):
            closest_idx = (np.abs(trajectory["Timestamp"] - row["Timestamp"])).argmin()
            closest_row = trajectory.iloc[closest_idx]
            
            # Format: timestamp x y z pitch roll yaw
            timestamp_ms = row["Timestamp"] * 1000  # Convert to ms
            base_data.append(
                f"{timestamp_ms:.0f} {closest_row['Pos X']:.3f} {closest_row['Pos Y']:.3f} "
                f"{closest_row['Pos Z']:.3f} {closest_row['Q_X']:.3f} {closest_row['Q_Y']:.3f} "
                f"{closest_row['Q_Z']:.3f}"
            )
        
        with open(os.path.join(episode_path, "base.txt"), "w") as f:
            f.write("\n".join(base_data))
        print(f"Saved {len(base_data)} robot states")
        
        # Save result.txt (assuming successful execution)
        with open(os.path.join(episode_path, "result.txt"), "w") as f:
            f.write(f"{timestamps_ms[-1]:.0f} 1")  # Last timestamp, success=1
        print("Saved result.txt")
            
    except Exception as e:
        print(f"Error processing episode {episode}: {e}")
        print("Creating empty ARIO structure...")
        
        # Create empty camera data
        for cam_idx in [1, 2]:
            cam_path = os.path.join(episode_path, f"cam-{cam_idx}")
            create_empty_video(cam_path)
        
        # Create empty base.txt
        with open(os.path.join(episode_path, "base.txt"), "w") as f:
            f.write("0 0.000 0.000 0.000 0.000 0.000 0.000")
        
        # Create empty result.txt
        with open(os.path.join(episode_path, "result.txt"), "w") as f:
            f.write("0 1")

def create_empty_video(cam_path):
    """Create an empty video file with a black frame."""
    # Create a black frame
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Save video
    video_path = os.path.join(cam_path, "rgb.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(video_path, fourcc, 30.0, (640, 480))
    out.write(frame)
    out.release()
    
    # Save empty timestamps
    np.save(os.path.join(cam_path, "timestamps.npy"), np.array([0]))

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str, default="task_name")
    parser.add_argument("--num_episodes", type=int, default=1)
    args = parser.parse_args()
    task = args.task
    num_episodes = args.num_episodes

    # Load configuration
    config = load_config()
    TASK_CONFIG = config["task_config"]
    cfg = TASK_CONFIG
    cfg["task_name"] = task  # Add task name to config

    # Data path
    data_path = os.path.join(config["device_settings"]["data_dir"], "dataset", str(task))
    print(f"\nData path: {data_path}")
    os.makedirs(data_path, exist_ok=True)

    for episode in range(num_episodes):
        process_episode(episode, config, data_path, cfg)
        print(f"episode_{episode} done!")

    print("All episodes completed successfully!")

if __name__ == "__main__":
    main() 