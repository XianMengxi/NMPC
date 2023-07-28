#!/bin/python3
import rospy
from ocs2_msgs.msg import mpc_target_trajectories, mpc_observation, mpc_state, mpc_input


# 创建一个mpc_observation对象，用于存储来自订阅者的最新观测数据
observation = mpc_observation()

def observation_callback(msg):
    # 在回调函数中更新全局变量observation的值为接收到的消息
    global observation
    observation = msg

# 创建一个名为"target_publisher"的ROS节点
rospy.init_node("target_publisher")

# 创建一个Publisher，用于发布目标轨迹数据到主题"/cartpole_mpc_target"
target_pub = rospy.Publisher("/cartpole_mpc_target", mpc_target_trajectories, queue_size=1)

# 创建一个Subscriber，接收来自"/cartpole/mpc_observer"主题的mpc_observation消息，并调用observation_callback函数进行处理
observation_sub = rospy.Subscriber("/cartpole/mpc_observer", mpc_observation, observation_callback)

def listen_to_keyboard():
    """从键盘读取用户输入，生成目标轨迹数据"""
    input_string = input("输入目标:【角度1 角度2 角度3 位置 时间】")
    input_string = input_string.split(" ")
    old_state = observation.state

    # 检查输入格式是否正确
    if len(input_string) != 5:
        print("输入格式不正确")
        return None

    theta1 = float(input_string[0])
    theta2 = float(input_string[1])
    theta3 = float(input_string[2])
    pos = float(input_string[3])
    time = float(input_string[4])

    # 检查时间是否合法
    if time <= 0:
        print("时间不对")
        return None

    if observation.time != 0:
        print("机器人开动")

        # 计算目标状态和输入
        target_time = observation.time + time
        target_state = mpc_state([pos, theta1, theta2, theta3, 0, 0])
        target_input = mpc_input([0])

        # 创建mpc_target_trajectories对象，包含了目标轨迹的时间、状态和输入
        target_trajectory = mpc_target_trajectories([observation.time, target_time],
                                                    [old_state, target_state],
                                                    [target_input, target_input])
        return target_trajectory
    else:
        print("机器人开都没开")
        return None


if __name__ == "__main__":
    while not rospy.is_shutdown():
        try:
            # 获取用户输入，生成目标轨迹数据
            user_input_trajectories = listen_to_keyboard()

            # 如果生成的目标轨迹数据不为空，则发布数据到主题"/cartpole_mpc_target"
            if user_input_trajectories != None:
                print("发布了轨迹：\n")
                print(user_input_trajectories)
                target_pub.publish(user_input_trajectories)
        except rospy.ROSInterruptException:
            break
