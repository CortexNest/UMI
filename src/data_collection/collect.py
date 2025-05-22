#!/usr/bin/env python3

import argparse
import csv
import json
import os
import threading
from collections import deque
from pathlib import Path
from time import sleep
from typing import Any, Dict

import cv2
import rospy
import yaml
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tqdm import tqdm


class DataCollector:
    """数据采集器类，用于从各种传感器收集数据"""

    def __init__(self, config_path: str, task: str, num_episodes: int):
        """
        初始化数据采集器

        Args:
            config_path: 配置文件路径
            task: 任务名称
            num_episodes: 采集的episode数量
        """
        self.project_root = self._get_project_root()
        self.config = self._load_config(config_path)
        self.task = task
        self.num_episodes = num_episodes

        # 设置任务配置
        self.TASK_CONFIG = self.config["task_config"]
        self.cfg = self.TASK_CONFIG
        self.EPISODE_LEN = self.cfg["episode_len"]  # record time (sec) = episode_len / 20

        # 设置数据存储路径
        self._setup_storage()

        # 初始化ROS相关变量
        self.video_subscriber = None
        self.trajectory_subscriber = None
        self.cv_bridge = CvBridge()
        self.video_buffer = deque()
        self.trajectory_buffer = deque()
        self.buffer_lock = threading.Lock()
        self.first_frame_timestamp = None
        self.start_time = 0
        self.first_time_judger = False

        # 初始化ROS节点
        rospy.init_node("video_trajectory_recorder", anonymous=True)

        # 视频写入器参数
        self.fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.frame_width = self.cfg["cam_width"]
        self.frame_height = self.cfg["cam_height"]

    def _get_project_root(self) -> Path:
        """获取项目根目录"""
        current_file = Path(__file__).resolve()
        return current_file.parent.parent.parent

    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """加载配置文件"""
        config_file = self.project_root / config_path
        if not config_file.exists():
            raise FileNotFoundError(f"配置文件不存在: {config_file}")

        if config_file.suffix == ".yaml":
            with open(config_file) as f:
                config = yaml.safe_load(f)
        elif config_file.suffix == ".json":
            with open(config_file) as f:
                config = json.load(f)
        else:
            raise ValueError(f"不支持的配置文件格式: {config_file.suffix}")

        return config

    def _setup_storage(self):
        """设置数据存储"""
        data_path = os.path.join(
            self.config["device_settings"]["data_dir"], "dataset", str(self.task)
        )
        os.makedirs(data_path, exist_ok=True)

        self.IMAGE_PATH = os.path.join(data_path, "camera/")
        os.makedirs(self.IMAGE_PATH, exist_ok=True)

        self.CSV_PATH = os.path.join(data_path, "csv/")
        os.makedirs(self.CSV_PATH, exist_ok=True)

        self.VIDEO_PATH_TEMP = os.path.join(self.IMAGE_PATH, "temp_video_n.mp4")
        self.TRAJECTORY_PATH_TEMP = os.path.join(self.CSV_PATH, "temp_trajectory_n.csv")
        self.TIMESTAMP_PATH_TEMP = os.path.join(self.CSV_PATH, "temp_video_timestamps_n.csv")
        self.FRAME_TIMESTAMP_PATH_TEMP = os.path.join(self.CSV_PATH, "frame_timestamps.csv")

    def video_callback(self, msg):
        """处理视频帧回调"""
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        timestamp = msg.header.stamp.to_sec()

        with self.buffer_lock:
            if self.start_time < timestamp:
                self.video_buffer.append((frame, timestamp))
                if self.first_time_judger:
                    self.first_frame_timestamp = timestamp
                    self.first_time_judger = False

    def trajectory_callback(self, msg):
        """处理轨迹数据回调"""
        timestamp = msg.header.stamp.to_sec()
        with self.buffer_lock:
            if self.start_time < timestamp:
                pose = msg.pose.pose
                self.trajectory_buffer.append(
                    (
                        timestamp,
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    )
                )

    def write_video(self, video_writer, timestamp_writer):
        """写入视频帧和时间戳"""
        frame_index = 0
        previous_progress = 0
        pbar = tqdm(total=self.EPISODE_LEN, desc="Processing Frames")

        while not rospy.is_shutdown():
            with self.buffer_lock:
                if self.video_buffer:
                    frame, timestamp = self.video_buffer.popleft()
                    video_writer.write(frame)
                    timestamp_writer.writerow([frame_index, timestamp])
                    frame_index += 1

                    current_progress = frame_index // 3
                    pbar.update(current_progress - previous_progress)
                    previous_progress = current_progress

                    if frame_index == 3 * self.EPISODE_LEN:
                        print("Video Done!")
                        pbar.close()
                        break

            sleep(0.001)

    def write_trajectory(self, trajectory_writer):
        """写入轨迹数据"""
        counter = 0
        while not rospy.is_shutdown():
            with self.buffer_lock:
                if self.trajectory_buffer:
                    (
                        Timestamp,
                        PosX,
                        PosY,
                        PosZ,
                        Q_X,
                        Q_Y,
                        Q_Z,
                        Q_W,
                    ) = self.trajectory_buffer.popleft()
                    trajectory_writer.writerow([Timestamp, PosX, PosY, PosZ, Q_X, Q_Y, Q_Z, Q_W])
                    counter += 1
                    if counter == 10 * self.EPISODE_LEN:
                        print("Trajectory Done!")
                        break

            sleep(0.001)

    def start_recording(self, video_writer, trajectory_writer, timestamp_writer):
        """开始记录数据"""
        self.video_buffer.clear()
        self.trajectory_buffer.clear()

        video_thread = threading.Thread(
            target=self.write_video, args=(video_writer, timestamp_writer)
        )
        trajectory_thread = threading.Thread(
            target=self.write_trajectory, args=(trajectory_writer,)
        )

        video_thread.start()
        trajectory_thread.start()

        video_thread.join()
        trajectory_thread.join()

    def run(self):
        """运行数据采集"""
        # 初始化订阅者
        self.video_subscriber = rospy.Subscriber(
            self.config["task_config"]["ros"]["video_topic"],
            Image,
            self.video_callback,
            queue_size=self.config["task_config"]["ros"]["queue_size"],
        )
        self.trajectory_subscriber = rospy.Subscriber(
            self.config["task_config"]["ros"]["trajectory_topic"],
            Odometry,
            self.trajectory_callback,
            queue_size=self.config["task_config"]["ros"]["queue_size"],
        )

        # 初始化帧时间戳文件
        with open(self.FRAME_TIMESTAMP_PATH_TEMP, "a", newline="") as frame_timestamp_file:
            frame_timestamp_writer = csv.writer(frame_timestamp_file)
            frame_timestamp_writer.writerow(["Episode Index", "Timestamp"])

            for episode in range(self.num_episodes):
                video_writer = cv2.VideoWriter(
                    self.VIDEO_PATH_TEMP.replace("_n", f"_{episode}"),
                    self.fourcc,
                    60,
                    (self.frame_width, self.frame_height),
                )

                # CSV文件用于轨迹数据和视频时间戳
                with open(
                    self.TRAJECTORY_PATH_TEMP.replace("_n", f"_{episode}"), "w", newline=""
                ) as trajectory_file, open(
                    self.TIMESTAMP_PATH_TEMP.replace("_n", f"_{episode}"), "w", newline=""
                ) as timestamp_file:
                    trajectory_writer = csv.writer(trajectory_file)
                    timestamp_writer = csv.writer(timestamp_file)

                    # 写入CSV文件头
                    trajectory_writer.writerow(
                        ["Timestamp", "Pos X", "Pos Y", "Pos Z", "Q_X", "Q_Y", "Q_Z", "Q_W"]
                    )
                    timestamp_writer.writerow(["Frame Index", "Timestamp"])

                    self.first_time_judger = False

                    input(
                        f"Episode {episode + 1}/{self.num_episodes} ready. Press Enter to start..."
                    )
                    self.start_time = rospy.Time.now().to_sec()
                    self.first_time_judger = True
                    print(f"Episode {episode + 1}/{self.num_episodes} started!")

                    try:
                        self.start_recording(video_writer, trajectory_writer, timestamp_writer)
                    except Exception as e:
                        print(f"An error occurred: {e}")
                        raise
                    finally:
                        frame_timestamp_writer.writerow([episode, self.first_frame_timestamp])
                        video_writer.release()

        print("All episodes completed successfully!")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=str,
        default="configs/data_collection.json",
        help="Path to configuration file",
    )
    parser.add_argument("--task", type=str, default="task_name", help="Task name")
    parser.add_argument("--num_episodes", type=int, default=1, help="Number of episodes to record")
    args = parser.parse_args()

    collector = DataCollector(args.config, args.task, args.num_episodes)
    collector.run()


if __name__ == "__main__":
    main()
