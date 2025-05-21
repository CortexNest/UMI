数据结构方案
数据文件结构整体分为：collection——series——task——episode，collection是指一次提交上传的数据集样本，可能包含不同的场景和机器人类型，series是指同一个场景和同一个机器人采集的系列数据，如双臂机器人在厨房采集的系列数据，可能包含不同的任务，task是一个具体的任务，比如抓取苹果，同一个任务可以重复采集多次，episode是针对某一具体任务的一次完整采集过程。episode下分传感器采集数据，各传感器可根据自己频率自行采集，但要以同一个时间戳为基准。示例文件结构如下：
collection（一次提交的数据集样本）
│  commit.yaml（提交者信息与声明）
│
├─series-1（同一个场景，同一个机器人）
│  │  calibration_cam-1.yaml（相机1标定参数）
│  │  calibration_cam-1_lidar-1.yaml（相机1与lidar 1的标定参数）
│  │  IMU.pdf（IMU传感器说明书）
│  │  information.yaml（场景描述，机器人信息，各传感器数量和信息）
│  │  touch.pdf（触觉传感器说明书）
│  │	 松灵机器人说明书.pdf
│  │
│  ├─task-1（一个任务，如：抓取苹果）
│  │  │  description.yaml（instruction）
│  │  │  task_record.mp4（每个任务的视频记录）
│  │  │
│  │  ├─episode-1（一次完整采集过程）
│  │  │  │  base.txt（机器人本体运动数据）
│  │  │  │  IMU-1.txt（IMU传感器数据）
│  │  │  │  left_master_arm_joint-0.txt（master左臂关节0数据）
│  │  │  │  left_master_gripper.txt（master左夹持器运动数据）
│  │  │  │  left_slave_arm_joint-0.txt（slave左臂关节0数据）
│  │  │  │  left_slave_gripper.txt（slave左夹持器运动数据）
│  │  │  │  pan_tilt.txt（头部云台数据）
│  │  │  │  right_master_arm_joint-5.txt（master右臂关节5数据）
│  │  │  │  right_master_gripper.txt（master右夹持器运动数据）
│  │  │  │  right_slave_arm_joint-5.txt（slave右臂关节5数据）
│  │  │  │  right_slave_gripper.txt（slave右臂的夹持器运动数据）
│  │  │  │  result.txt（样本执行结果）
│  │  │  │
│  │  │  ├─audio-1（音频数据，根据时间戳分片）
│  │  │  │      1709554382234_compressed.npz
│  │  │  │      1709554383638_compressed.npz
│  │  │  │
│  │  │  ├─cam-1（camera 1采的图像）
│  │  │  │      rgb.mp4
│  │  │  │      timestamps.npy
│  │  │  │
│  │  │  ├─cam-2
│  │  │  │      rgb.mp4
│  │  │  │      timestamps.npy
│  │  │  │
│  │  │  ├─lidar-1（激光雷达1采的点云，xyz单位：m）
│  │  │  │      1709554382234.npy
│  │  │  │      1709554382334.npy
│  │  │  │
│  │  │  ├─lidar-2
│  │  │  │      1709554382235.npy
│  │  │  │      1709554382354.npy
│  │  │  │
│  │  │  ├─rgbd-1（rgbd 1采的点云）
│  │  │  │      rgb.mp4
│  │  │  │      d.npz
│  │  │  │      timestamps.npy
│  │  │  │
│  │  │  └─touch-1（触觉传感器1的数据）
│  │  │          1709554382234.txt
│  │  │
│  │  └─episode-2
│  └─task-2
│      │  description.yaml
│      │  task_record.mp4
│      │
│      └─episode-1
│
└─series-2
│  information.yaml
│	 松灵机器人2说明书.pdf
    │
    └─task-1
        │  description.yaml
        │  task_record.mp4
        │
        └─episode-1


1.2 数据采样方案
    （1） 场景。数据采样场景宜多样化，示例场景如，室内：bedroom, kitchen, shopping mall, dining room, cafe, living room等，室外：park, residential district等。
    （2） 动作。机器人动作宜多样化，示例动作如：pick、move、open、close、push、place、put、navigate、separate、point、insert、knock、drag、drop、wipe、assemble、turn on等。
    （3） 必须采集。各模态的数据中，文本指令、图像是每次采集都必须有的，如果是操作相关的任务还必须采集机器人末端和夹持器状态数据，如果是导航相关的任务，则必须采集机器人本体运动状态数据，其他数据在有条件的情况下应尽量采集。
    （4） 默认单位。数据记录的时间戳，是Unix 时间戳，单位是ms，各传感器应使用同一时间戳基准。角度单位默认是度。单臂机器人的手臂，可以视为右臂。位移单位默认是m。力矩（扭矩）单位是Nm。浮点数一般保留小数点后3-4位。
    （5） task。针对同一个task，比如抓取物体，可以采集多个episode，可以将物体以不同姿态，放在桌上的不同位置，甚至可以调整机器人本体位置，分别采集对应的episode，建议一个task采集的episode数量不少于10次。
    （6） 视频记录。建议每个task用手机或相机以第三人称视角，针对某个episode，采集一个task_record.mp4，记录task相关的环境和操作信息，方便理解。
    （7） episode执行结果。每个episode的执行结果，是成功还是失败，需要在result.txt中记录，格式为：episode结束时间戳，执行结果状态（成功为1，失败为-1），中间用空格分隔。在一些仿真的环境中，以抓取物体为例，可能进行了几次抓取失败的尝试，这几次数据都采样到1个episode中，有些仿真软件能实时判断任务是否成功，这种情况可以按时间戳进行采集，每行记录一次采集结果，格式为：时间戳，执行结果状态（成功为1，失败为-1，执行中间过程为0），用空格分隔，最后一行的结果代表整个episode的结果。
    （8） 相机数据采集。以相机1为例，在cam-1目录下，每帧图像以时间戳命名，格式为png，或把图像内容按时间顺序以rgb.mp4的视频文件的格式保存，一般为uint8单通道或者3通道，时间戳以整形数组或列表的格式保存为timestamps.npy文件。information.yaml文件中提到的相机数据都要采集。采集帧率建议不低于30 FPS。
    （9） 激光雷达点云数据采集。以lidar 1为例，在lidar-1目录下，每帧点云以时间戳命名，格式为以numpy数组的方式np.float32数据类型保存成npy文件，数组shape为[N,3]，N表示点的个数，3表示xyz坐标，xyz单位是m。information.yaml文件中提到的lidar数据都要采集。采集频率不低于5 Hz。
    （10） rgbd相机数据采集。以rgbd 1相机为例，在rgbd-1目录下，保存方式有2种，优先推荐②：①将rgb和d分别按时间戳顺序保存为mp4视频，这是针对d的类型为uint8的情况，注意会有一定的失真，时间戳以整型数组或列表的格式保存为timestamps.npy，这种方法最节省空间，但d的误差较大。②将rgb保存为mp4视频，针对d的数据类型为uint8、uint16的情况，将d按时间戳顺序合成为一个数组，保存为d.npz，参考指令：np.savez_compressed(os.path.join(rgbd_path, 'd.npz'), d=d_npz)；如果d的数据类型为float32，建议量化到uint8或uint16，量化系数保存在information.yaml，量化后的数据再按前述uint8、uint16的情况保存，时间戳以整型数组或列表的格式保存为timestamps.npy，这种方法能大幅节省空间，经部分数据分析，量化到uint8，depth的平均精度损失约1.3cm，存储空间减小到原来的0.5%-2%，量化到uint16，depth的平均精度损失约6e-5m，存储空间减小到原来的5%-20%。information.yaml文件中提到的rgbd数据都要采集。采集帧率建议不低于30 FPS，深度d的默认单位是m。
    （11） 触摸传感器数据采集。以touch 1为例，在touch-1目录下，每帧数据以时间戳命名。information.yaml文件中提到的touch数据都要采集。
    （12） 本体移动数据的采集。记录在base.txt中，格式为：每行记录一次采集数据，每行按顺序依次为时间戳，剩下的严格按照information.yaml 中base_control的设置依次采集，各变量用空格隔开。采集频率最好不低于30 Hz。
    （13） 手臂末端夹持器的数据采集。左右手分别记录在left_master_gripper.txt和right_master_gripper.txt，若额外有2条slave手臂，记录文件命名可以拓展为left_slave_gripper.txt，right_slave_gripper.txt。单臂的只记录right_master_gripper.txt，格式为：每行记录一次采集数据，每行按顺序依次为时间戳，剩下的严格按照information.yaml 中endpoint_control的设置依次采集，最后加上夹持器开合状态，各变量用空格隔开。夹持器打开是0，闭合是1，若有中间状态则映射为0-1。采集频率最好不低于30 Hz。
    （14） 音频数据的采集。在episode目录下，有多少个录音设备就创建多少个audio文件夹，录音是整个episode过程持续采集，主要是采集机器人在运动或操作过程中与环境触碰的声音，比如夹持器碰到杯子的声音，保存文件夹命名格式为“audio-设备序号”，如：audio-1，在audio-1目录下，每帧音频片段以时间戳_compressed.npz命名保存。参考代码：np.savez_compressed(os.path.join(episode_path, 'audio-1', str(cur_time)+'_compressed.npz'), audio=data)，其中data为numpy一维数组，表示该时间戳对应的音频片段。
    （15） 手臂关节数据的采集。information.yaml文件中，若recorded_left_master_arm_joints等相关变量不为空，则应采集相应关节的数据，例如left_master_arm_joint-0.txt，采集的是master左臂0号关节的数据，格式为：每行记录一次采集数据，每行按顺序依次为时间戳，剩下的严格按照information.yaml 中joint_control的设置依次采集，变量用空格隔开。其他手臂和关节数据可类比采集。采集频率最好不低于30 Hz。
    （16） 头部云台数据的采集。information.yaml文件中，若pan_tilt为True，则应采集机器人头部云台的数据，例如pan_tilt.txt，格式为：每行记录一次采集数据，每行按顺序依次为时间戳，剩下的严格按照information.yaml 中pan_tilt_control的设置依次采集，各变量用空格隔开。
    （17） IMU数据的采集。information.yaml文件中，若IMU_num大于0，则应采集IMU的数据，例如IMU-1.txt，格式为：每行记录一次采集数据，每行按顺序依次为时间戳，加速度x，加速度y，加速度z，角速度x，角速度y，角速度z，各变量用空格隔开，加速度单位为m/s2，角速度单位为度/s。同时应在“information.yaml”文件同一目录下附上IMU数据手册，如：'IMU.pdf'，并在机器人说明书上描述IMU的xyz正方向对应的机器人方向。
    （18） 关于人形机器人或四足机器狗等形态的数据采集。本体中心的运动，可以视为base，参考base.txt的格式采集，其他如腿部/腰部等数据，可以参考末端执行器或手臂关节的格式采集，以“left_foot.txt”、“left_foot_joint-0.txt”等命名数据文件。
    （19） 其他数据的采集。对于一些通过仿真平台采集的数据，建议在每个episode中增加仿真环境中的物体和机器人的初始位姿坐标的记录，保存到auxiliary.json，建议以字典的数据类型保存每个物体的初始位置坐标和姿态欧拉角，方便仿真的复现，具体格式不做强制要求。推荐作者上传仿真数据相关的文件，如模型文件、脚本文件、仿真工具等，但不做强制要求。
    （20） 命名规范。各数据文件和目录的命名应严格遵循格式要求，可以参考示例文件，方便后续数据处理。
    （21） 说明书。机器人平台相关的说明文档，可附在collection（各series用同一款机器人）或series（各series用不同机器人）目录下。
    （22） 咨询。有问题可以发邮件咨询yeh@pcl.ac.cn。


1.3 配置文件详解
1.3.1 commit.yaml
    • author_name: 提交人姓名，如：'David'
    • work_organization: 提交人单位，如：'PCL'
    • author_email: 提交人邮箱，如：'yh1David@pcl.ac.cn'
    • role: 提交人职位，如：'engineer'
    • dataset_name: 此次提交数据集名称，如：'songling'
    • license: 数据集开源license，应支持在署名情况下可商用，可填：'CC BY 4.0'，'MIT license'，'Unsure'
    • PII_exclude: 数据集是否排除了可识别个人身份的信息，必须为True，相关责任由上传者承担
    • thirdparty_consent: 如果数据集包含了第三方数据，请确保得到第三方授权，必须为True，相关责任由上传者承担
    • healthy_content: 数据集内容应确保健康，不能包含涉及暴力、毒品、虐待等不健康主题的内容，必须为True，相关责任由上传者承担

1.3.2 information.yaml
    • series_name: 系列名称，可根据平台、场景、agent形态命名，如：'Songling_kitchen_bimanual'
    • scene: 场景描述，室内/室外，厨房/客厅……，如：'indoor, kitchen'
    • is_simulated: 真实数据还是仿真数据，True表示仿真，False表示真实
    • morphology: 机器人形态，可多选，如：['bimanual','wheeled']，候选范围：
-'bimanual'：双臂
-'single_arm'：单臂
-'AGV'：自动导引车
-'quadrupedal'：四足的
-'wheeled'：轮式的
-'drone'：无人机
-'humanoid'：人形机器人
    • num_joints_per_arm: 每条手臂的关节数，不含夹持器，范围>=0,没有手臂的填0
    • gripper: 夹持器的控制是张开/闭合二元状态还是连续状态，候选范围：
-'binary'：二元的
-'continuous'：连续的
    • same_coordinate: 对于采集的执行器末端数据或者本体移动数据，其坐标系方向是否与预设坐标系方向相同，如相同，则为'True'，注意类型为str，不是bool。预设坐标系如下图所示，机器人右边为x正方向，前方为y正方向，头顶上方为z正方向，坐标系原点的位置在右臂关节的肩部，即右手臂与身体的连接部位的中心。机器人坐标系可以与预设坐标系不同，但必须都是遵循右手定则的坐标系，并且x-pitch，y-roll，z-yaw，这三者的对应关系应一致，即不能x对应roll。数据提供方应尽可能了解机器人坐标系方向，否则会降低数据的使用价值。候选范围：
-'True'：机器人坐标系与预设坐标系相同
-'False'：机器人坐标系与预设坐标系不同
-'Unknown'：不了解当前机器人坐标系

    • endpoint_transform和origin_offset: 针对机械手操作任务，若same_coordinate为'False'，则需指定末端执行器的坐标系到预设坐标系的旋转矩阵和平移坐标，如：endpoint_transform为[[1,0,0],[0,1,0],[0,0,1]]，origin_offset为[0,0,0]。计算公式为：采集的末端坐标·endpoint_transform + origin_offset = 预设坐标系下的末端坐标。若same_coordinate为'Unknown'，endpoint_transform可设置为[]，但强烈建议了解机器人坐标系后提供旋转矩阵，否则会降低数据的使用价值。origin_offset如不能提供精确值（误差小于1cm），可以不提供，设置为：[]，没有手臂的机器人也为：[]。机器人说明书里应附上末端执行器坐标系的方向。
    • base_transform: 针对本体需要移动的任务，若same_coordinate为'False'，则需指定本体移动的坐标系到预设坐标系的转换矩阵，如：[[1,0,0],[0,1,0],[0,0,1]]。计算公式为：采集的本体移动坐标·base_transform = 预设坐标系下的本体移动坐标。机器人说明书里应附上本体移动坐标系的方向。若same_coordinate为'Unknown'，base_transform可设置为[]，但强烈建议了解本体坐标系后提供转换矩阵，否则会降低数据的使用价值。
    • IMU_transform: 针对有IMU且本体需要移动的任务，若same_coordinate为'False'，则需指定IMU的坐标系到预设坐标系的转换矩阵，如：[[1,0,0],[0,1,0],[0,0,1]]。计算公式为：采集的IMU坐标·IMU_transform = 预设坐标系下的IMU坐标。机器人说明书里应附上IMU坐标系的方向。若same_coordinate为'Unknown'，IMU_transform可设置为[]。
    • endpoint_control: 执行器末端运动坐标形式，从如下可选范围中挑选要采集的数据内容，left_gripper.txt、right_gripper.txt中每行的数据内容与顺序，除首位的时间戳和末位的开合状态外，应严格与endpoint_control的内容相对应，如：{'relative':['x', 'y', 'z'],'absolute':['pitch', 'roll', 'yaw']}，表示要以相对值记录末端的xyz，绝对值记录pitch，roll，yaw角度。可选范围：
-'absolute'/'relative'：绝对值/相对值，绝对值是指每次采集的运动数据是基于预设坐标系或其他本体坐标系的值，相对值是指，每次采集的运动数据都是在上一次运动基础上的变化值。这个概念只针对位置和姿态，一般速度、角速度、力矩都是'absolute'。
- 'x', 'y', 'z'：位置坐标
- 'pitch', 'roll', 'yaw'：姿态，'pitch', 'roll', 'yaw'依次分别对应xyz轴的旋转
-'vx', 'vy', 'vz'：xyz方向的运动速度
-'wx', 'wy', 'wz'：xyz方向的旋转速度
-'tx', 'ty', 'tz'：xyz方向的力矩
-'none'：没有手臂，或无法移动，或不记录相关数据
    • base_control: 机器人本体运动坐标形式，从可选范围中挑选要采集的数据内容，base.txt中每行的数据内容与顺序，除首位的时间戳外，应严格与base_control的内容相对应，如： {'relative':['x', 'y', 'yaw']}，表示要以相对值记录本体的xy位移和yaw方向旋转角度。又如：{'absolute':['vx', 'vy', 'wz']}，表示要记录xy方向的速度和绕z旋转角速度。可选范围参考endpoint_control。
    • joint_control: 机器人手臂关节运动形式，从可选范围中挑选要采集的数据内容，left_arm_joint-0.txt等关节数据文件中，每行的数据内容与顺序，除首位的时间戳外，应严格与joint_control的内容相对应，如：{'absolute':['yaw', 'wz', 'tz']}，表示要以绝对值记录关节的角度、角速度、力矩，由于关节只有1个自由度，默认都是z方向。可选范围参考endpoint_control。
    • pan_tilt: 机器人头部是可动云台则为True,否则为False。
    • pan_tilt_control：若pan_tilt为True，则需设置头部云台运动坐标形式，从可选范围中挑选要采集的数据内容，pan_tilt.txt中每行的数据内容与顺序，除首位的时间戳外，应严格与pan_tilt_control的内容相对应，如：{'absolute':['pitch', 'yaw']} ，表示要以绝对值记录云台的俯仰角和偏航角。坐标系应按照预设坐标系的规定。可选范围参考endpoint_control。
    • endpoint_origin: 若末端执行器用的是绝对坐标，这里指明坐标原点在机器人上的位置，可选范围：
-'shoulder'：手臂根部
-'middle'：机器人本体中部
-'head'：机器人头部
-'bottom'：机器人底部
-'environment'：原点不在机器人身上，而是在环境物体的某个位置
-'none'：非绝对坐标或不涉及此信息
    • action_frequency: 机器人控制动作频率，单位：Hz，如：30
    • blocking_control: 是否阻塞式控制，即必须先完成当前指令的动作才能执行下一个指令，True表明是阻塞式
    • arm_num：要采集的机器人手臂数量，如：4，手臂数量超过2，应在机器人说明书上描述各手臂与人之间的操控关系。
    • arm_operation_mode: 机器人手臂操作模式，列表长度应等于手臂数量，元素顺序为：[master左，master右，slave左，slave右，...]，如：['kinesthetic', 'kinesthetic', 'teleoperation', 'teleoperation']，元素可选范围：
-'kinesthetic'：人直接把机械手移到指定位置或把机器人推到指定位置，如Aloha的主手
-'manipulation'：人通过遥控器操作
-'teleoperation'：人手上或身上有运动传感器采集人动作数据，机器人跟随采集数据运动，如Aloha的从手，或者通过VR装备控制
-'imitation'：机器人通过激光雷达/相机等传感器学习/模仿人动作，机器人和人没有直接接触
-'generation'：通过算法或工具直接生成机器人运动数据，不需要人操作
    • operation_mode：机器人操作模式，适用于没有机械臂的机器人，比如导航机器人，如['manipulation'] ，元素可选范围同上。
    • camera_num: 相机数量，有多少个相机，下面就要根据示例填多少个相机的信息，如：2
    • cam_view: 相机安装视角，列表长度应等于相机数量，如：['ego-centric','third-person']，可选范围：
-'ego-centric'：第一人称，一般指安装在头顶的相机
-'third-person'：第三人称，一般指安装在桌子或周围环境的相机
-'left_wrist'：安装在机器人左臂腕部或执行器末端的相机
- 'right_wrist'：安装在机器人右臂腕部或执行器末端的相机
    • cam_calibration_file: 相机的标定参数文件，列表长度应等于相机数量，如：['calibration_cam-1.yaml','']，参数文件放在与information.yaml相同的路径，没有标定文件可以为：[]
    • cam_file_type: 图像文件保存格式，如：'png'，详情见1.2（8）相机数据采集。可选范围：
-'png'：png图片格式保存
-'mp4'：'rgb.mp4'视频格式保存，时间戳以整型数组或列表格式保存为'timestamps.npy'文件
    • lidar_num: 激光雷达数量，有多少个激光雷达，下面就要根据示例填多少个激光雷达的信息，如：2
    • lidar_position: 激光雷达安装位置，列表长度应等于雷达数量，如：['head', 'bottom']，可选范围：
-'head'：机器人头部
-'middle'：机器人中间
-'bottom'：机器人底部
    • cam1_lidar1_calibration_file: 相机1和lidar1的相对位置标定参数文件，如：'calibration_cam-1_lidar-1.yaml'，文件放在与information.yaml相同的路径，如无，可填：''。如有其他相机或lidar的相对位置标定文件也可参考此格式填上，如：cam2_lidar2_calibration_file: 'calibration_cam-2_lidar-2.yaml'。
    • rgbd_num: rgbd相机数量，有多少个rgbd相机，下面就要根据示例填写多少个rgbd相机信息，如：1
    • rgbd_view: rgbd相机安装视角，列表长度应等于相机数量，如：['third-person']，可选范围：
-'ego-centric'：第一人称，一般指安装在头顶的相机
-'third-person'：第三人称，一般指安装在桌子或周围环境的相机
-'left_wrist'：安装在机器人左臂腕部或执行器末端的相机
- 'right_wrist'：安装在机器人右臂腕部或执行器末端的相机
    • rgbd_file_type: rgbd文件保存格式，如：'mp4_u16npz'，详情见1.2（10）rgbd相机数据采集，可选范围：
-'mp4'：rgb数据保存为'rgb.mp4'文件，depth数据（uint8类型）保存为'd.mp4'文件，会有一定失真，时间戳以整型数组或列表格式保存为'timestamps.npy'文件
-'mp4_u16npz'：rgb数据保存为'rgb.mp4'文件，depth数据（uint16类型）按时间戳顺序合成为一个数组，保存为'd.npz'文件，时间戳以整型数组或列表格式保存为'timestamps.npy'文件
-'mp4_u8npz'：depth数据为uint8类型，其余同上
    • depth_max：针对将rgbd中的depth由float32转为uint8或uint16的情况，这里记录1个series下所有episode的所有rgbd的原float32的depth数值的最大值，为减小转换误差，建议保留到小数点后9位数字。
    • depth_min：针对将rgbd中的depth由float32转为uint8或uint16的情况，这里记录1个series下所有episode的所有rgbd的原float32的depth数值的最小值，为减小转换误差，建议保留到小数点后9位数字。
    • depth_w：针对将rgbd中的depth由float32转为uint8或uint16的情况，这里记录1个series下所有episode的所有rgbd的已经转为uint8或uint16的depth变换回float32的系数，变换公式：dfloat32 = duint * depth_w + depth_b，为减小转换误差，建议保留到小数点后9位数字。
    • depth_b：上面公式中的depth_b，为减小转换误差，建议保留到小数点后9位数字。
    • touch_num: 触摸传感器数量，有多少个传感器，下面就要根据示例填多少个信息，如：2
    • touch_position: 触摸传感器安装位置，列表长度应等于传感器数量，如：['left, gripper','right, gripper']
    • touch_manual: 触摸传感器说明书，如用了触摸传感器，应在“information.yaml”文件同一目录下附上说明书，如：'touch.pdf'，如无，可填：''
    • recorded_left_master_arm_joints: 要采集运动数据的master左臂关节点序号，这里不包括夹持器的运动数据，这里的序号要跟实际采集的数据一致，数值越大，表示该关节越靠近末端，如：[0,1,2,3,4,5]，如不采集关节信息，可为：[]。
    • recorded_left_slave_arm_joints：要采集运动数据的slave左臂关节点序号，详情同上，如果只有1条左臂，则此项为[]。
    • recorded_right_master_arm_joints: 要采集运动数据的master右臂关节点序号,说明同上，如：[0,1,2,3,4,5]。
    • recorded_right_slave_arm_joints：要采集运动数据的slave右臂关节点序号，详情同上，如果只有1条右臂，则此项为[]。
    • audio_num: 音频传感器数量，如：1。
    • IMU_num: IMU数量，有多少个IMU，下面就要根据示例填多少个IMU的信息，如：1。
    • IMU_position: IMU 安装位置，列表长度应等于IMU数量，如：['bottom']，可选范围：
-'head'：机器人头部
-'middle'：机器人中间
-'bottom'：机器人底部
    • incomplete:是否存在部分数据缺失的情况，True表示有缺失，False表示完整，如果没有这个参数，默认为完整。有缺失的例子比如有3个相机（rgbd），部分episode可能只有2个或1个相机（rgbd）的数据。

1.3.3 description.yaml
    • instruction_EN: 给机器人的英文动作指令，指令应与机器人动作一致，必填，如：'pick up the apple from the table and put it into the bowl'
    • instruction_CH：给机器人的中文动作指令，意思应与英文指令一致，可以不填，如：'将苹果从桌子上拿起，放入碗中'，或不填留空：''
    • skills: instruction 中涉及的机器人技能，就是其中的动词，列表类型，如：['pick', 'put']