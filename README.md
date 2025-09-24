# 环境配置
## 方式1
mkdir ws_ur_single

cd ws_ur_single

git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git src/ur_simulation_gz

- 替换src/ur_simulation_gz/ur_simulation_gz.humble.repos

cd ws_ur_single

- 下载所需要的包
vcs import < Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos

## 方式2
压缩包里面是已经下载的包，适用于u22-humble

- 按照文件结构截图把下载的包分散在不同文件夹，每个都在ws目录执行rosdepc自动下载所需要的包
rosdepc update
rosdepc install --ignore-src --from-paths src -y 

- 对于ws_ur_single和ws_ur_multi，在rosdepc后还需要编译
colcon build --symlink-install 
source install/setup.bash


# 运行指令
- 单机械臂 Rviz+ Gazebo 
ros2 launch ur_simulation_gz ur_sim_control.launch.py  
- 单机械臂 MoveIt + Gazebo 
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py  

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e
ros2 run joint_state_pkg joint_state_subscriber
