ROBOT_CONFIG = {
    'robot_ip': "192.168.1.18",
    'robot_port': 8080,
    'robot_type': 'Realman_Gen72',
    'robot_joint_num': 7,
    'original_joints' : [-0.433, -34.477, -0.003, -126.484, 5.022, 93.425, -0.036], # degree
    'original_gripper_width': 80, # 0: close, 100: open
    'gripper_open_width': 80, # 0: close, 100: open
    'gripper_close_width': 20, # 0: close, 100: open
    'gripper_threshold': 0.5, # 0: close, 1: open
}
CAMERA_CONFIG = {
    'camera_list': ['front'], # name list
    'input_image_width': 640,
    'input_image_height': 360,
}