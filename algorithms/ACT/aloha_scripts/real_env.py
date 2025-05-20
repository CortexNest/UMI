import time
import numpy as np
import collections
import dm_env
from Robotic_Arm.rm_robot_interface import *

import IPython
e = IPython.embed

class ImageRecorder:
    def __init__(self, init_node=True, is_debug=False):
        from collections import deque
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        self.is_debug = is_debug
        self.bridge = CvBridge()
        self.camera_names = ['front']

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
            rospy.Subscriber("/usb_cam/image_raw", Image, callback_func) # hardcode
            if self.is_debug:
                setattr(self, f'{cam_name}_timestamps', deque(maxlen=50))
                
        time.sleep(0.5)

    def image_cb(self, cam_name, data):
        setattr(self, f'{cam_name}_image', self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8'))
        setattr(self, f'{cam_name}_secs', data.header.stamp.secs)
        setattr(self, f'{cam_name}_nsecs', data.header.stamp.nsecs)
        if self.is_debug:
            getattr(self, f'{cam_name}_timestamps').append(data.header.stamp.secs + data.header.stamp.secs * 1e-9)

    def image_cb_cam_front(self, data):
        cam_name = 'front'
        return self.image_cb(cam_name, data)

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
    Action space:      [arm_qpos (7),             # absolute joint position
                        gripper_positions (1),    # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ arm_qpos (7),         # absolute joint position
                                        gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)
                        "images": "front": (1080x1920x3) }    # h, w, c, dtype='uint8'
    """

    def __init__(self):
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.arm.rm_create_robot_arm("192.168.1.18", 8080)
        self.setup_robots()
        self.image_recorder = ImageRecorder(init_node=False)

    def setup_robots(self):
        # 设置末端电压为 24V
        if self.arm.rm_set_tool_voltage(3) == 0:
            print("设置电压成功")
        else:
            raise Exception("设置电压失败")
        # 配置末端通讯接口为ModbusRTU模式
        if self.arm.rm_set_modbus_mode(1,115200,3) == 0:
            print("设置ModbusRTU模式成功")
        else:
            raise Exception("设置ModbusRTU模式失败")

    def get_qpos(self):
        # 获取机械臂当前关节角度
        ok, joint = self.arm.rm_get_joint_degree()
        if ok != 0:
            raise Exception("获取关节角度失败: ", ok)
        # 获取当前夹爪宽度
        read_param = rm_peripheral_read_write_params_t(1, 40000, 1)
        ok, width = self.arm.rm_read_holding_registers(read_param)
        if ok != 0:
            raise Exception("获取夹爪宽度失败: ", ok)
        width = width / 100.0 # normalize gripper width
        return np.concatenate(joint, width)

    def get_images(self):
        return self.image_recorder.get_images()

    def set_joints(self, joint):
        ok = self.arm.rm_movej(joint, 20, 0, 0, 1)
        if ok != 0:
            raise Exception("运动到指定关节角度失败: ", ok)

    def set_gripper(self, width):
        write_params = rm_peripheral_read_write_params_t(1, 40000, 1)
        ok = self.arm.rm_write_single_register(write_params, width)
        if ok != 0:
            raise Exception("设置夹爪宽度失败: ", ok)

    def get_observation(self):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['images'] = self.get_images()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            self.set_joints([0.4099999964237213, -23.413999557495117, 0.9409999847412109, -108.58599853515625, 1.3029999732971191, 90.21900177001953, 0.004000000189989805])
            self.set_gripper(0)
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

    def step(self, action):
        self.set_joints(action[:7])
        self.set_gripper(action[7] * 100.0) # unnormalize gripper width
        time.sleep(0.02)
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

def make_real_env():
    env = RealEnv()
    return env

