## 1. 环境安装

```
conda create -n act python=3.9
conda activate act
pip install torchvision
pip install torch
pip install pyquaternion
pip install pyyaml
pip install rospkg
pip install pexpect
pip install mujoco==2.3.7
pip install dm_control==1.0.14
pip install opencv-python
pip install matplotlib
pip install einops
pip install packaging
pip install h5py
pip install ipython
pip install Robotic_Arm==1.0.1
cd src/training/ACT/detr && pip install -e .
```

## 2. 模型训练
    
    python3 src/training/ACT/train.py \
    --task_name <task_name> \
    --ckpt_dir <ckpt dir> \
    --policy_class ACT --kl_weight 10 --chunk_size 20 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 \
    --num_epochs 20000  --lr 1e-5 \
    --seed 0

## 模型推理

启动 roscore:

    roscore

启动 GoPro 的 ROS 节点
```
roslaunch usb_cam usb_cam-test.launch
```
运行脚本，使用训练相同参数
```
python3 src/inference/ACT/infer.py \
--task_name <task_name> \
--ckpt_dir <ckpt dir> \
--policy_class ACT --kl_weight 10 --chunk_size 20 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 \
--num_epochs 20000  --lr 1e-5 \
--seed 0 --num_rollout 1 --temporal_agg
```
To enable temporal ensembling, add flag ``--temporal_agg``.

For real-world data where things can be harder to model, train for at least 5000 epochs or 3-4 times the length after the loss has plateaued.
Please refer to [tuning tips](https://docs.google.com/document/d/1FVIZfoALXg_ZkYKaYVh-qOlaXveq5CtvJHXkY25eYhs/edit?usp=sharing) for more info.
