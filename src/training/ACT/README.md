## 训练
    
    python3 src/training/ACT/train.py \
    --task_name <task_name> \
    --ckpt_dir <ckpt dir> \
    --policy_class ACT --kl_weight 10 --chunk_size 20 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 \
    --num_epochs 20000  --lr 1e-5 \
    --seed 0

## 推理

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
