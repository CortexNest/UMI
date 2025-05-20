### Task parameters
DATA_DIR = '/home/panjing/project/act/dataset'
TASK_CONFIGS = {
    'close_ricecooker_scripted':{
        'dataset_dir': DATA_DIR + '/close_ricecooker_test_joint',
        'num_episodes': 20,
        'episode_len': 120,
        'camera_names': ['front']
    },
}