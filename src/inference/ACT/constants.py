### Task parameters
DATA_DIR = 'your dataset path'
TASK_CONFIGS = {
    'grasp_corn':{
        'dataset_dir': DATA_DIR + '/grasp_corn_joint_for_real_v0',
        'num_episodes': 68,
        'episode_len': 180,
        'camera_names': ['front']
    },
    'grasp_cube':{
        'dataset_dir': DATA_DIR + '/grasp_cube_joint_for_real_v0',
        'num_episodes': 81,
        'episode_len': 180,
        'camera_names': ['front']
    },
}