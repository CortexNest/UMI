import time
import cv2
import numpy as np
import collections
from Robotic_Arm.rm_robot_interface import *
from collections import deque
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# from threading import Lock
from realman_scripts.robot_config import ROBOT_CONFIG, CAMERA_CONFIG

ROBOT_IP = ROBOT_CONFIG['robot_ip']
ROBOT_PORT = ROBOT_CONFIG['robot_port']
START_JOINT = ROBOT_CONFIG['original_joints']
START_WIDTH = ROBOT_CONFIG['original_gripper_width']
OPEN_WIDTH = ROBOT_CONFIG['gripper_open_width']
CLOSE_WIDTH = ROBOT_CONFIG['gripper_close_width']
GRIPPER_THRESHOLD = ROBOT_CONFIG['gripper_threshold']

CAMERA_LIST = CAMERA_CONFIG['camera_list']
IMAGE_WIDTH = CAMERA_CONFIG['input_image_width']
IMAGE_HEIGHT = CAMERA_CONFIG['input_image_height']

class ImageRecorder:
    def __init__(self, init_node=True, is_debug=False):

        self.is_debug = is_debug
        self.bridge = CvBridge()
        self.camera_names = CAMERA_LIST

        if init_node:
            rospy.init_node('image_recorder', anonymous=True)

        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)
            if cam_name == 'front':
                callback_func = self.image_cb_cam_front
            else:
                raise NotImplementedError
            rospy.Subscriber("/usb_cam/image_raw", Image, callback_func, queue_size=1000)
            if self.is_debug:
                setattr(self, f'{cam_name}_timestamps', deque(maxlen=50))
                
        time.sleep(1)

    def image_cb(self, cam_name, data):
        # img_np = np.frombuffer(data.data, dtype=np.uint8)
        # img_np = img_np.reshape((data.height, data.width, -1))
        # if data.encoding in ("yuv422", "yuyv"):
        #     img_bgr = cv2.cvtColor(img_np, cv2.COLOR_YUV2BGR_YUYV)
        # else:
        #     img_bgr = img_np
        # image = cv2.resize(img_bgr, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_LINEAR)
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        raw_img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8') # 1080x1920x3
        image = cv2.resize(np.array(raw_img), (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_LINEAR) # 360x640x3
        setattr(self, f'{cam_name}_image', image)
        setattr(self, f'{cam_name}_secs', data.header.stamp.secs)
        setattr(self, f'{cam_name}_nsecs', data.header.stamp.nsecs)
        if self.is_debug:
            getattr(self, f'{cam_name}_timestamps').append(data.header.stamp.secs + data.header.stamp.secs * 1e-9)

    def image_cb_cam_front(self, data):
        cam_name = 'front'
        self.image_cb(cam_name, data)

    def get_images(self):
        image_dict = dict()
        for cam_name in self.camera_names:
            image_dict[cam_name] = getattr(self, f'{cam_name}_image')
        return image_dict

    def print_diagnostics(self):
        def dt_helper(l):
            l = np.array(l)
            diff = l[1:] - l[:-1]
            return np.mean(diff)
        for cam_name in self.camera_names:
            image_freq = 1 / dt_helper(getattr(self, f'{cam_name}_timestamps'))
            print(f'{cam_name} {image_freq=:.2f}')
        print()

class RealEnv:
    """
    Action space:      [arm_qpos (7),             # absolute joint (degrees)
                        gripper_positions (1),    # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ arm_qpos (7),         # absolute joint (degrees)
                                        gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)
                        "images": "front": (360x640x3) }    # h, w, c, bgr, dtype='uint8'
    """
    def __init__(self):
        self.arm = None
        self.handle = None
        self.image_recorder = ImageRecorder(init_node=True)
        # self.data_lock = Lock()
        self.cur_gripper_width = 1.0
        self.cur_joints = START_JOINT
        self.cnt = 0

    def setup_robots(self):
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.arm.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT)

        if self.arm.rm_set_tool_voltage(3) == 0:
            print("set voltage success")
        else:
            raise Exception("set voltage failed")
        
        if self.arm.rm_set_modbus_mode(1,115200,2) == 0:
            print("set modbus success")
        else:
            raise Exception("set modbus failed")

        # config = rm_realtime_push_config_t(2, True, 8089, 0, "192.168.1.104")
        # self.arm.rm_set_realtime_push(config)
        # ok, _ = self.arm.rm_get_realtime_push()
        # if ok == 0:
        #     print("set realtime push success")
        # else:
        #     raise Exception("set realtime push failed")
        # arm_state_callback = rm_realtime_arm_state_callback_ptr(self.state_callback)
        # self.arm.rm_realtime_arm_state_call_back(arm_state_callback)

    def stop_robots(self):
        # self.arm.rm_close_modbus_mode(1)
        self.arm.rm_delete_robot_arm()

    # def state_callback(self, data):
    #     with self.data_lock:
    #         self.cur_joints = data.joint_status.joint_position[:]

    def get_qpos(self):
        ok, joint = self.arm.rm_get_joint_degree()
        if ok != 0:
            joint = self.cur_joints
            print("get joint degree failed: ", ok)

        # read_param = rm_peripheral_read_write_params_t(1, 40005, 1)
        # ok, width = self.arm.rm_read_holding_registers(read_param)
        # if ok != 0:
        #     width = self.cur_gripper_width
        #     print("get gripper width failed: ", ok)
        # width = width / 100.0 * 9.1 / 8.3 # normalize gripper width

        # with self.data_lock:
        #     joint = self.cur_joints[:]

        width = self.cur_gripper_width
        joint.append(width)
        return joint

    def get_images(self):
        return self.image_recorder.get_images()

    def set_joints(self, joint):
        ok = self.arm.rm_movej(joint, 30, 0, 0, 1)
        if ok != 0:
            print("set joint failed: ", ok)

    def set_joints_canfd(self, joint):
        self.cur_joints = joint
        ok = self.arm.rm_movej_canfd(joint, False)
        if ok != 0:
            print("set joint failed: ", ok)

    def set_gripper(self, width):
        # self.arm.rm_set_modbus_mode(1, 115200, 2)
        write_params = rm_peripheral_read_write_params_t(1, 40000, 1)
        ok = self.arm.rm_write_single_register(write_params, width)
        if ok != 0:
            print("set gripper width failed: ", ok)
        # self.arm.rm_close_modbus_mode(1)

    def set_gripper_open_or_close(self, width):
        if width < GRIPPER_THRESHOLD:
            if self.cur_gripper_width > GRIPPER_THRESHOLD:
                # self.arm.rm_set_modbus_mode(1, 115200, 2)
                write_params = rm_peripheral_read_write_params_t(1, 40000, 1)
                ok = self.arm.rm_write_single_register(write_params, CLOSE_WIDTH)
                if ok != 0:
                    print("set gripper width failed: ", ok)
                time.sleep(1)
                # self.arm.rm_close_modbus_mode(1)
                self.cur_gripper_width = 0.0
        else:
            if self.cur_gripper_width < GRIPPER_THRESHOLD:
                # self.arm.rm_set_modbus_mode(1, 115200, 2)
                write_params = rm_peripheral_read_write_params_t(1, 40000, 1)
                ok = self.arm.rm_write_single_register(write_params, OPEN_WIDTH)
                if ok != 0:
                    print("set gripper width failed: ", ok)
                time.sleep(1)
                # self.arm.rm_close_modbus_mode(1)
                self.cur_gripper_width = 1.0

    def get_observation(self):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['images'] = self.get_images()
        return obs

    def reset(self):
        self.cnt = 0
        self.cur_gripper_width = 1.0
        self.cur_joints = START_JOINT
        self.set_joints(START_JOINT)
        time.sleep(1)
        self.set_gripper(START_WIDTH)
        time.sleep(1)
        return self.get_observation()

    def step(self, action):
        self.cnt += 1
        print("step: ", self.cnt)
        self.set_joints_canfd(action[:7])
        time.sleep(0.02)
        self.set_gripper_open_or_close(action[-1])
        return self.get_observation()
