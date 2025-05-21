from Robotic_Arm.rm_robot_interface import *
import time

class RobotArmController:
    def __init__(self, ip, port=8080, level=3, mode=2):
        """
        初始化并连接机器人
        
        Args:
            ip (str): 机器人的IP地址
            port (int): 端口号，默认8080
            level (int): 连接等级，默认3
            mode (int): 线程模式，默认2 (0:单线程, 1:双线程, 2:三线程)
        """
        self.thread_mode = rm_thread_mode_e(mode)
        self.robot = RoboticArm(self.thread_mode)
        self.handle = self.robot.rm_create_robot_arm(ip, port, level)

        if self.handle.id == -1:
            print("\n连接机器人失败\n")
            exit(1)
        else:
            print(f"\n成功连接到机器人: {self.handle.id}\n")
            
    def get_current_pose(self):
        """
        获取机器人当前状态
        
        Returns:
            tuple: (ret, state) 
                - ret: 返回码 (0表示成功)
                - state: 包含位姿信息的字典
        """
        ret, state = self.robot.rm_get_current_arm_state()
        return ret, state

    def move_joint(self, joint_angles, speed=20, radius=0, block=1):
        """
        执行关节运动
        
        Args:
            joint_angles (list): 目标关节角度列表
            speed (int): 关节速度，默认20度/秒
            radius (float): 交融半径，默认0
            block (int): 是否阻塞，1为阻塞，0为非阻塞
        
        Returns:
            int: 返回码，0表示成功
        """
        return self.robot.rm_movej(joint_angles, speed, radius, 0, block)
    
    def disconnect(self):
        """
        断开与机器人的连接
        """
        handle = self.robot.rm_delete_robot_arm()
        if handle == 0:
            print("\n成功断开机器人连接\n")
        else:
            print("\n断开机器人连接失败\n")

def print_joint_angles(joint_angles):
    """
    打印关节角度信息
    
    Args:
        joint_angles (list): 关节角度列表
    """
    print("关节角度:")
    for i, angle in enumerate(joint_angles):
        print(f"关节 {i+1}: {angle:.2f}°")

def check_joint_angles(current_angles, target_angles, tolerance=0.5):
    """
    检查关节角度是否达到目标位置
    
    Args:
        current_angles (list): 当前关节角度列表
        target_angles (list): 目标关节角度列表
        tolerance (float): 允许的误差（度）
        
    Returns:
        bool: 是否所有关节都在允许误差范围内
    """
    for i, (current, target) in enumerate(zip(current_angles, target_angles)):
        if abs(current - target) > tolerance:
            print(f"关节 {i+1} 未达到目标位置: 目标 {target:.2f}°, 当前 {current:.2f}°")
            return False
    return True

def main():
    try:
        robot = RobotArmController("192.168.33.80")
        
        # 获取当前关节角度
        print("获取当前关节角度...")
        ret, state = robot.get_current_pose()
        if ret == 0:
            current_joints = state["joint"]
            print("\n当前关节角度:")
            print_joint_angles(current_joints)
            
            # 设置目标关节角度
            target_joints = [
                0,      # 第1关节: 0度
                -90,    # 第2关节: -90度
                0,      # 第3关节: 0度
                -150,   # 第4关节: -150度
                0,      # 第5关节: 0度
                60,     # 第6关节: 60度
                0       # 第7关节: 0度
            ]
            
            print("\n目标关节角度:")
            print_joint_angles(target_joints)
            
            # 执行关节运动
            print("\n开始移动到目标位置...")
            # 使用较低的速度以确保安全
            ret = robot.move_joint(target_joints, speed=10)  # 速度设为10度/秒
            
            if ret == 0:
                print("\n运动完成")
                # 验证最终位置
                ret, state = robot.get_current_pose()
                if ret == 0:
                    final_joints = state["joint"]
                    print("\n最终关节角度:")
                    print_joint_angles(final_joints)
                    
                    # 检查所有关节是否到达目标位置
                    if check_joint_angles(final_joints, target_joints):
                        print("\n所有关节已成功到达目标位置")
                    else:
                        print("\n警告：部分关节未完全到达目标位置")
            else:
                print(f"\n运动失败，错误码: {ret}")
        else:
            print(f"获取机器人状态失败，错误码: {ret}")
            
    except Exception as e:
        print(f"发生异常: {str(e)}")
        
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()