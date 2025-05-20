import os
import json
import cv2
import argparse
from tqdm import tqdm
from time import sleep
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import csv
import threading
from collections import deque

# Load configuration from config.json
with open('config/config.json', 'r') as f:
    config = json.load(f)

TASK_CONFIG = config['task_config']
cfg = TASK_CONFIG
EPISODE_LEN = cfg['episode_len'] # record time (sec) = episode_len / 20

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--task', type=str, default="grasp_cube_v1_24")  # task name
parser.add_argument('--num_episodes', type=int, default=6) # episodes num
args = parser.parse_args()
task = args.task
num_episodes = args.num_episodes

data_path = os.path.join(config['device_settings']["data_dir"], "dataset" ,str(task))
os.makedirs(data_path, exist_ok=True)

IMAGE_PATH = os.path.join(data_path, 'camera/')
os.makedirs(IMAGE_PATH, exist_ok=True)

CSV_PATH = os.path.join(data_path, 'csv/')
os.makedirs(CSV_PATH, exist_ok=True)

VIDEO_PATH_TEMP = os.path.join(data_path, 'camera', 'temp_video_n.mp4')
TRAJECTORY_PATH_TEMP = os.path.join(data_path, 'csv', 'temp_trajectory_n.csv')
TIMESTAMP_PATH_TEMP = os.path.join(data_path, 'csv', 'temp_video_timestamps_n.csv')
FRAME_TIMESTAMP_PATH_TEMP = os.path.join(data_path, 'csv', 'frame_timestamps.csv')

video_subscriber = None
trajectory_subscriber = None

# Initialize ROS node
rospy.init_node('video_trajectory_recorder', anonymous=True)

# Video writer parameters for 60 Hz recording
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
frame_width, frame_height = cfg['cam_width'], cfg['cam_height']

# Buffers for storing incoming data
video_buffer = deque()
trajectory_buffer = deque()

# Lock for thread synchronization
buffer_lock = threading.Lock()

# Initialize CvBridge for image conversion
cv_bridge = CvBridge()

# Variable to store the first frame's timestamp
first_frame_timestamp = None

# Callback for video frames (60 Hz expected)
def video_callback(msg):
    global first_frame_timestamp, first_time_judger
    frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    timestamp = msg.header.stamp.to_sec()

    with buffer_lock:
        if start_time < timestamp:
            video_buffer.append((frame, timestamp))
            if first_time_judger:
                first_frame_timestamp = timestamp
                first_time_judger = False

# Callback for trajectory data (e.g., T265 at 200 Hz)
def trajectory_callback(msg):
    timestamp = msg.header.stamp.to_sec()  # Ensure timestamp is in Unix format (float)
    with buffer_lock:
        if start_time < timestamp:
            pose = msg.pose.pose
            trajectory_buffer.append((timestamp, pose.position.x, pose.position.y, pose.position.z,
                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

# Thread for writing video frames and timestamps
def write_video():
    frame_index = 0
    previous_progress = 0  # Keep track of the last progress value
    pbar = tqdm(total=EPISODE_LEN, desc='Processing Frames')

    while not rospy.is_shutdown():
        with buffer_lock:
            if video_buffer:
                frame, timestamp = video_buffer.popleft()
                video_writer.write(frame)

                # Write timestamp for each frame to CSV
                timestamp_writer.writerow([frame_index, timestamp])
                frame_index += 1

                # Update progress bar
                current_progress = frame_index // 3
                pbar.update(current_progress - previous_progress)
                previous_progress = current_progress

                if frame_index == 3 * EPISODE_LEN:
                    print("Video Done!")
                    pbar.close()  # Close the progress bar
                    break

        sleep(0.001)  # Small sleep to avoid CPU overload

# Thread for writing trajectory data to CSV
def write_trajectory():
    counter = 0
    while not rospy.is_shutdown():
        with buffer_lock:
            if trajectory_buffer:
                Timestamp, PosX, PosY, PosZ, Q_X, Q_Y, Q_Z, Q_W = trajectory_buffer.popleft()
                trajectory_writer.writerow([Timestamp, PosX, PosY, PosZ, Q_X, Q_Y, Q_Z, Q_W])
                counter += 1
                if counter == 10 * EPISODE_LEN:
                    print("Trajectory Done!")
                    break

        sleep(0.001)  # Small sleep to avoid CPU overload

# Main function to start recording
def start_recording():
    global video_subscriber, trajectory_subscriber
    # Subscribe to video and trajectory topics
    video_buffer.clear()
    trajectory_buffer.clear()

    # Start separate threads for writing video and trajectory data
    video_thread = threading.Thread(target=write_video)
    trajectory_thread = threading.Thread(target=write_trajectory)

    video_thread.start()
    trajectory_thread.start()

    video_thread.join()
    trajectory_thread.join()

if __name__ == "__main__":
    # Initialize subscribers
    start_time = 0
    cv_bridge = CvBridge()
    video_subscriber = rospy.Subscriber(config['task_config']['ros']['video_topic'], Image, video_callback, queue_size=config['task_config']['ros']['queue_size'])
    trajectory_subscriber = rospy.Subscriber(config['task_config']['ros']['trajectory_topic'], Odometry, trajectory_callback, queue_size=config['task_config']['ros']['queue_size'])

    # Initialize frame timestamp file
    with open(FRAME_TIMESTAMP_PATH_TEMP, "a", newline='') as frame_timestamp_file:
        frame_timestamp_writer = csv.writer(frame_timestamp_file)
        frame_timestamp_writer.writerow(['Episode Index', 'Timestamp'])

        for episode in range(num_episodes):
            video_writer = cv2.VideoWriter(VIDEO_PATH_TEMP.replace("_n", f"_{episode}"), fourcc, 60, (frame_width, frame_height))

            # CSV for trajectory data and video timestamps
            with open(TRAJECTORY_PATH_TEMP.replace("_n", f"_{episode}"), 'w', newline='') as trajectory_file, \
                 open(TIMESTAMP_PATH_TEMP.replace("_n", f"_{episode}"), 'w', newline='') as timestamp_file:

                trajectory_writer = csv.writer(trajectory_file)
                timestamp_writer = csv.writer(timestamp_file)

                # Write headers to CSV files
                trajectory_writer.writerow(['Timestamp', 'Pos X', 'Pos Y', 'Pos Z', 'Q_X', 'Q_Y', 'Q_Z', 'Q_W'])
                timestamp_writer.writerow(['Frame Index', 'Timestamp'])

                first_time_judger = False

                input(f"Episode {episode + 1}/{num_episodes} ready. Press Enter to start...")
                start_time = rospy.Time.now().to_sec()  # Start time
                first_time_judger = True
                print(f"Episode {episode + 1}/{num_episodes} started!")

                # Start recording
                try:
                    start_recording()
                except Exception as e:
                    print(f"An error occurred: {e}")
                    raise  # Reraise exception to terminate execution
                finally:
                    frame_timestamp_writer.writerow([episode, first_frame_timestamp])
                    video_writer.release()

    print("All episodes completed successfully!")