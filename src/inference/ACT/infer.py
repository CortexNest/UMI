import argparse
import csv
import os
import pickle
import sys
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import torch
from einops import rearrange

sys.path.append(os.path.join(str(Path(__file__).parent.parent.parent), "robot_control"))
sys.path.append(os.path.join(str(Path(__file__).parent.parent.parent), "training"))
sys.path.append(os.path.join(str(Path(__file__).parent.parent.parent), "training", "ACT"))
from datetime import datetime

from ACT.policy import ACTPolicy, CNNMLPPolicy
from ACT.utils import set_seed
from constants import TASK_CONFIGS
from realman_scripts.real_env import RealEnv


def main(args):
    set_seed(args["seed"])
    # command line parameters
    ckpt_dir = args["ckpt_dir"]
    policy_class = args["policy_class"]
    task_name = args["task_name"]
    num_epochs = args["num_epochs"]

    # get task parameters
    is_sim = False
    task_config = TASK_CONFIGS[task_name]
    episode_len = task_config["episode_len"]
    camera_names = task_config["camera_names"]

    # fixed parameters
    state_dim = 8
    lr_backbone = 1e-5
    backbone = "resnet18"
    if policy_class == "ACT":
        enc_layers = 4
        dec_layers = 7
        nheads = 8
        policy_config = {
            "lr": args["lr"],
            "num_queries": args["chunk_size"],
            "kl_weight": args["kl_weight"],
            "hidden_dim": args["hidden_dim"],
            "dim_feedforward": args["dim_feedforward"],
            "lr_backbone": lr_backbone,
            "backbone": backbone,
            "enc_layers": enc_layers,
            "dec_layers": dec_layers,
            "nheads": nheads,
            "camera_names": camera_names,
        }
    elif policy_class == "CNNMLP":
        policy_config = {
            "lr": args["lr"],
            "lr_backbone": lr_backbone,
            "backbone": backbone,
            "num_queries": 1,
            "camera_names": camera_names,
        }
    else:
        raise NotImplementedError

    config = {
        "num_epochs": num_epochs,
        "ckpt_dir": ckpt_dir,
        "episode_len": episode_len,
        "state_dim": state_dim,
        "lr": args["lr"],
        "policy_class": policy_class,
        "policy_config": policy_config,
        "task_name": task_name,
        "seed": args["seed"],
        "temporal_agg": args["temporal_agg"],
        "camera_names": camera_names,
        "num_rollout": args["num_rollout"],
    }

    # eval phase
    eval_bc(config, f"policy_best.ckpt")


def make_policy(policy_class, policy_config):
    if policy_class == "ACT":
        policy = ACTPolicy(policy_config)
    elif policy_class == "CNNMLP":
        policy = CNNMLPPolicy(policy_config)
    else:
        raise NotImplementedError
    return policy


def make_optimizer(policy_class, policy):
    if policy_class == "ACT":
        optimizer = policy.configure_optimizers()
    elif policy_class == "CNNMLP":
        optimizer = policy.configure_optimizers()
    else:
        raise NotImplementedError
    return optimizer


def get_image(img_list, camera_names):
    curr_images = []
    for cam_name in camera_names:
        curr_image = rearrange(img_list[cam_name], "h w c -> c h w")
        curr_images.append(curr_image)
    curr_image = np.stack(curr_images, axis=0)
    curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
    return curr_image


def eval_bc(config, ckpt_name):
    seed = config["seed"]
    ckpt_dir = config["ckpt_dir"]
    state_dim = config["state_dim"]
    policy_class = config["policy_class"]
    policy_config = config["policy_config"]
    camera_names = config["camera_names"]
    max_timesteps = config["episode_len"]
    temporal_agg = config["temporal_agg"]
    num_rollout = config["num_rollout"]
    set_seed(seed)

    # load policy and stats
    ckpt_path = os.path.join(ckpt_dir, ckpt_name)
    policy = make_policy(policy_class, policy_config)
    loading_status = policy.load_state_dict(torch.load(ckpt_path))
    print(loading_status)
    policy.cuda()
    policy.eval()
    print(f"Loaded: {ckpt_path}")
    stats_path = os.path.join(ckpt_dir, f"dataset_stats.pkl")
    with open(stats_path, "rb") as f:
        stats = pickle.load(f)

    # preprocess and postprocess
    pre_process = lambda s_qpos: (s_qpos - stats["qpos_mean"]) / stats["qpos_std"]
    post_process = lambda a: a * stats["action_std"] + stats["action_mean"]

    try:
        # load environment
        env = RealEnv()
        env.setup_robots()

        query_frequency = policy_config["num_queries"]  # num_queries == chunk_size
        if temporal_agg:  # temporal aggregation
            query_frequency = 1
            num_queries = policy_config["num_queries"]

        max_timesteps = int(max_timesteps * 2)  # may increase for real-world tasks

        image_history = []
        qpos_history = []
        target_qpos_history = []
        for rollout_id in range(num_rollout):
            input(f"Rollout {rollout_id + 1}/{num_rollout} ready. Press Enter to start...")

            ### reset environment
            obs = env.reset()

            ### evaluation loop
            if temporal_agg:
                all_time_actions = torch.zeros(
                    [max_timesteps, max_timesteps + num_queries, state_dim]
                ).cuda()

            image_list = []
            qpos_list = []
            target_qpos_list = []
            with torch.inference_mode():
                for t in range(max_timesteps):
                    ### process previous timestep to get qpos and image_list
                    qpos_numpy = np.array(obs["qpos"])
                    qpos = pre_process(qpos_numpy)
                    qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
                    curr_image = get_image(obs["images"], camera_names)

                    image_list.append(obs["images"]["front"])
                    qpos_list.append(obs["qpos"])

                    ### query policy
                    if config["policy_class"] == "ACT":
                        if t % query_frequency == 0:
                            all_actions = policy(qpos, curr_image)
                        if temporal_agg:
                            all_time_actions[[t], t : t + num_queries] = all_actions
                            actions_for_curr_step = all_time_actions[:, t]
                            actions_populated = torch.all(actions_for_curr_step != 0, axis=1)
                            actions_for_curr_step = actions_for_curr_step[actions_populated]
                            k = 0.01
                            exp_weights = np.exp(-k * np.arange(len(actions_for_curr_step)))
                            exp_weights = exp_weights / exp_weights.sum()
                            exp_weights = torch.from_numpy(exp_weights).cuda().unsqueeze(dim=1)
                            raw_action = (actions_for_curr_step * exp_weights).sum(
                                dim=0, keepdim=True
                            )
                        else:
                            raw_action = all_actions[:, t % query_frequency]
                    elif config["policy_class"] == "CNNMLP":
                        raw_action = policy(qpos, curr_image)
                    else:
                        raise NotImplementedError

                    ### post-process actions
                    raw_action = raw_action.squeeze(0).cpu().numpy()
                    action = post_process(raw_action)
                    target_qpos = action.tolist()

                    ### step the environment
                    obs = env.step(target_qpos)

                    target_qpos_list.append(target_qpos)

            print(f"Rollout {rollout_id + 1}/{num_rollout} finished")

            image_history.append(image_list)
            qpos_history.append(qpos_list)
            target_qpos_history.append(target_qpos_list)

    finally:
        # close environment
        env.stop_robots()
        print("Environment closed")

        # save images and qpos
        current_time = datetime.now().strftime("%Y_%m_%d_%H_%M")
        save_path = f"./saved_data/{config['task_name']}/{current_time}"
        os.makedirs(save_path, exist_ok=True)
        for i in range(len(image_history)):
            images_path = os.path.join(save_path, f"image_list_{i}")
            os.makedirs(images_path, exist_ok=True)
            video_writer = cv2.VideoWriter(
                os.path.join(save_path, f"video_{i}.mp4"),
                cv2.VideoWriter_fourcc(*"mp4v"),
                20,
                (640, 360),
            )
            for j, image_np in enumerate(image_history[i]):
                video_writer.write(image_np)
                image_path = os.path.join(images_path, f"image_{j}.png")
                cv2.imwrite(image_path, image_np)
            video_writer.release()

        for i in range(len(qpos_history)):
            qpos_path = os.path.join(save_path, f"qpos_{i}.csv")
            with open(qpos_path, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "joint_0",
                        "joint_1",
                        "joint_2",
                        "joint_3",
                        "joint_4",
                        "joint_5",
                        "joint_6",
                        "gripper width",
                    ]
                )
                for j in range(len(qpos_history[i])):
                    writer.writerow(qpos_history[i][j])

        for i in range(len(target_qpos_history)):
            target_qpos_path = os.path.join(save_path, f"target_qpos_{i}.csv")
            with open(target_qpos_path, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "joint_0",
                        "joint_1",
                        "joint_2",
                        "joint_3",
                        "joint_4",
                        "joint_5",
                        "joint_6",
                        "gripper width",
                    ]
                )
                for j in range(len(target_qpos_history[i])):
                    writer.writerow(target_qpos_history[i][j])

        print(f"Saved all images and qpos")


def forward_pass(data, policy):
    image_data, qpos_data, action_data, is_pad = data
    image_data, qpos_data, action_data, is_pad = (
        image_data.cuda(),
        qpos_data.cuda(),
        action_data.cuda(),
        is_pad.cuda(),
    )
    return policy(qpos_data, image_data, action_data, is_pad)  # TODO remove None


def plot_history(train_history, validation_history, num_epochs, ckpt_dir, seed):
    # save training curves
    for key in train_history[0]:
        plot_path = os.path.join(ckpt_dir, f"train_val_{key}_seed_{seed}.png")
        plt.figure()
        train_values = [summary[key].item() for summary in train_history]
        val_values = [summary[key].item() for summary in validation_history]
        plt.plot(np.linspace(0, num_epochs - 1, len(train_history)), train_values, label="train")
        plt.plot(
            np.linspace(0, num_epochs - 1, len(validation_history)), val_values, label="validation"
        )
        # plt.ylim([-0.1, 1])
        plt.tight_layout()
        plt.legend()
        plt.title(key)
        plt.savefig(plot_path)
    print(f"Saved plots to {ckpt_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt_dir", action="store", type=str, help="ckpt_dir", required=True)
    parser.add_argument(
        "--policy_class", action="store", type=str, help="policy_class, capitalize", required=True
    )
    parser.add_argument("--task_name", action="store", type=str, help="task_name", required=True)
    parser.add_argument("--batch_size", action="store", type=int, help="batch_size", required=True)
    parser.add_argument("--seed", action="store", type=int, help="seed", required=True)
    parser.add_argument("--num_epochs", action="store", type=int, help="num_epochs", required=True)
    parser.add_argument("--lr", action="store", type=float, help="lr", required=True)

    # for ACT
    parser.add_argument("--kl_weight", action="store", type=int, help="KL Weight", required=False)
    parser.add_argument("--chunk_size", action="store", type=int, help="chunk_size", required=False)
    parser.add_argument("--hidden_dim", action="store", type=int, help="hidden_dim", required=False)
    parser.add_argument(
        "--dim_feedforward", action="store", type=int, help="dim_feedforward", required=False
    )
    parser.add_argument("--temporal_agg", action="store_true")

    parser.add_argument("--num_rollout", action="store", type=int, default=1, required=False)

    main(vars(parser.parse_args()))
