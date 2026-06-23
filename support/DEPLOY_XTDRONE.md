# 风力发电机模型部署到 XTDrone 操作步骤

## 环境基线

```
ROS:       Noetic
Gazebo:    11.15.1
工作空间:   /home/lste/catkin_ws
模型路径:   /home/lste/.gazebo/models/fans_model
XTDrone:   /home/lste/XTDrone
PX4:       /home/lste/PX4_Firmware
```

已有环境变量 (在 /home/lste/.bashrc 中)：
```bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/lste/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/lste/PX4_Firmware/Tools/sitl_gazebo
```

---

## 操作 1：修复 planning_context.launch 的 xacro 问题

**问题**：MoveIt! 的 `planning_context.launch` 用 `command="xacro"` 解析 URDF，但 SW2URDF 导出的 `.xacro` 文件实际是已展开的纯 URDF（不含任何 xacro 宏），xacro 解析会报错。

**修改文件**：`fans_moveit_config/launch/planning_context.launch` 第 9 行

```xml
<!-- 修改前 -->
<param if="$(arg load_robot_description)" name="$(arg robot_description)"
  command="xacro  '$(find wind_generator_description)/urdf/wind_generator.xacro'"/>

<!-- 修改后 -->
<param if="$(arg load_robot_description)" name="$(arg robot_description)"
  textfile="$(find wind_generator_description)/urdf/wind_generator.xacro"/>
```

---

## 操作 2：将 4 个 ROS 包链接到 catkin 工作空间

```bash
ln -s /home/lste/.gazebo/models/fans_model/wind_generator_description /home/lste/catkin_ws/src/
ln -s /home/lste/.gazebo/models/fans_model/wind_generator_gazebo /home/lste/catkin_ws/src/
ln -s /home/lste/.gazebo/models/fans_model/fans_move /home/lste/catkin_ws/src/
ln -s /home/lste/.gazebo/models/fans_model/fans_moveit_config /home/lste/catkin_ws/src/
```

之后编译：
```bash
cd /home/lste/catkin_ws && catkin_make
```

---

## 操作 3：创建世界文件

**新建文件**：`/home/lste/PX4_Firmware/Tools/sitl_gazebo/worlds/wind_farm.world`

```xml
<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 1000 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane><normal>0 0 1</normal><size>200 200</size></plane>
          </geometry>
          <surface>
            <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>
            <contact><ode/></contact>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane><normal>0 0 1</normal><size>200 200</size></plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>

    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>

    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>

    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
  </world>
</sdf>
```

**为什么放在这里**：XTDrone 的 launch 文件通过 `$(find mavlink_sitl_gazebo)` 引用世界文件，该包解析到 `/home/lste/PX4_Firmware/Tools/sitl_gazebo/`。

---

## 操作 4：创建 XTDrone 集成启动文件

**新建文件**：`/home/lste/PX4_Firmware/launch/wind_generator_demo.launch`

```xml
<?xml version="1.0"?>
<launch>
  <!-- World and Gazebo settings -->
  <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/wind_farm.world"/>
  <arg name="gui" default="true"/>
  <arg name="paused" default="false"/>
  <arg name="debug" default="false"/>
  <arg name="verbose" default="true"/>

  <!-- Wind generator spawn position -->
  <arg name="fan_x" default="0"/>
  <arg name="fan_y" default="0"/>
  <arg name="fan_z" default="0"/>

  <!-- Drone options -->
  <arg name="spawn_drone" default="false"/>
  <arg name="drone_x" default="30"/>
  <arg name="drone_y" default="0"/>
  <arg name="drone_z" default="10"/>
  <arg name="drone_vehicle" default="iris"/>
  <arg name="drone_sdf" default="iris_realsense_camera"/>
  <!-- drone_ns must match {vehicle}_{ID} for XTDrone keyboard control -->
  <arg name="drone_ns" default="iris_0"/>

  <!-- ============ 1. Start Gazebo ============ -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(arg world)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="verbose" value="$(arg verbose)"/>
    <arg name="paused" value="$(arg paused)"/>
  </include>

  <!-- ============ 2. Spawn Wind Generator (URDF) ============ -->
  <param name="robot_description"
    textfile="$(find wind_generator_description)/urdf/wind_generator.xacro"/>

  <node name="spawn_wind_generator" pkg="gazebo_ros" type="spawn_model"
    args="-urdf -param robot_description -model wind_generator
          -x $(arg fan_x) -y $(arg fan_y) -z $(arg fan_z)"
    respawn="false" output="screen"/>

  <!-- ============ 3. ROS Controllers ============ -->
  <rosparam file="$(find fans_moveit_config)/config/ros_controllers.yaml" command="load"/>

  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" args="fans_controller joint_state_controller"/>

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    respawn="true" output="screen"/>

  <!-- ============ 4. Spawn Drone (optional) ============ -->
  <group if="$(arg spawn_drone)">
    <group ns="$(arg drone_ns)">
      <arg name="ID" value="0"/>
      <arg name="ID_in_group" value="0"/>
      <arg name="fcu_url" default="udp://:24540@localhost:34580"/>

      <include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">
        <arg name="x" value="$(arg drone_x)"/>
        <arg name="y" value="$(arg drone_y)"/>
        <arg name="z" value="$(arg drone_z)"/>
        <arg name="R" value="0"/>
        <arg name="P" value="0"/>
        <arg name="Y" value="0"/>
        <arg name="vehicle" value="$(arg drone_vehicle)"/>
        <arg name="sdf" value="$(arg drone_sdf)"/>
        <arg name="mavlink_udp_port" value="18570"/>
        <arg name="mavlink_tcp_port" value="4560"/>
        <arg name="ID" value="$(arg ID)"/>
        <arg name="ID_in_group" value="$(arg ID_in_group)"/>
      </include>

      <include file="$(find mavros)/launch/px4.launch">
        <arg name="fcu_url" value="$(arg fcu_url)"/>
        <arg name="gcs_url" value=""/>
        <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
        <arg name="tgt_component" value="1"/>
      </include>
    </group>
  </group>
</launch>
```

**为什么放在这里**：XTDrone 的 launch 文件统一放在 `/home/lste/PX4_Firmware/launch/`，由 `$(find px4)` 解析。

---

## 操作 5：安装缺失的 ROS 依赖

```bash
sudo apt install -y \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-position-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-robot-state-publisher
```

**检查安装结果**：
```bash
rospack find joint-trajectory-controller
rospack find position-controllers
rospack find joint-state-controller
rospack find gazebo-ros-control
```

---

## 操作 6：修复 fans_move.py 执行权限

```bash
chmod +x /home/lste/catkin_ws/src/fans_move/scripts/fans_move.py
```

---

## 操作 7：翻转 URDF 关节轴方向

仅改轨迹正负号不能可靠地反向，根本方法是翻转 URDF 中 `fans_joint` 的旋转轴。

**修改文件**：`wind_generator_description/urdf/wind_generator.xacro` 第 59 行

```xml
<!-- 修改前 -->
<axis xyz="0 0 -1" />

<!-- 修改后 -->
<axis xyz="0 0 1" />
```

同时将 `fans_move.py` 的轨迹目标改回正数，与新轴正方向一致：

**修改文件**：`fans_move/scripts/fans_move.py` 第 22-23 行

```python
# 最终版本 (axis=0 0 1, 正向旋转)
JointTrajectoryPoint(positions=[0.0], velocities=[0.0], ...),
JointTrajectoryPoint(positions=[10000], velocities=[0.2], ...),
```

> 注意：URDF 变更后必须重启 Gazebo 才能生效。

---

## 操作 8：修复命名空间对齐

**问题**：初始版本把无人机放在 `ns="uav0"` 下，但 XTDrone 的键盘控制脚本固定发布到 `/xtdrone/iris_0/cmd_vel_flu`，通讯桥梁也固定订阅 `iris_0/mavros/...`，命名空间不匹配导致控制链断裂。

**修改文件**：`/home/lste/PX4_Firmware/launch/wind_generator_demo.launch`

```xml
<!-- 修改前 -->
<group ns="uav0">

<!-- 修改后 -->
<arg name="drone_ns" default="iris_0"/>
<group ns="$(arg drone_ns)">
```

**验证方法**：
```bash
# 启动仿真后检查话题
rostopic list | grep iris_0
# 应看到 iris_0/mavros/... 话题
```

---

## 操作 9：重写 fans_moveit_config/gazebo.launch

**修改文件**：`fans_moveit_config/launch/gazebo.launch`

改为使用 `wind_farm.world`，确保仅启动风力发电机时（不带无人机）也能正常工作：

```xml
<?xml version="1.0"?>
<launch>
  <arg name="paused" default="false"/>
  <arg name="gazebo_gui" default="true"/>
  <arg name="urdf_path" default="$(find wind_generator_description)/urdf/wind_generator.xacro"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find mavlink_sitl_gazebo)/worlds/wind_farm.world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gui" value="$(arg gazebo_gui)"/>
  </include>

  <param name="robot_description" textfile="$(arg urdf_path)"/>

  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model"
    args="-urdf -param robot_description -model wind_generator -x 0 -y 0 -z 0"
    respawn="false" output="screen"/>

  <include file="$(find fans_moveit_config)/launch/ros_controllers.launch"/>
</launch>
```

---

## 操作 10：创建瞬间停止脚本

**新建文件**：`fans_move/scripts/fans_stop.py`

```python
#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction

def main():
    rospy.init_node('trajectory_stopper')

    client = actionlib.SimpleActionClient(
        'fans_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for joint trajectory action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to joint trajectory action server")

    rospy.loginfo("Cancelling all goals...")
    client.cancel_all_goals()
    rospy.loginfo("Fan stopped.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

**添加执行权限**：
```bash
chmod +x /home/lste/catkin_ws/src/fans_move/scripts/fans_stop.py
```

**原理**：通过 actionlib 的 `cancel_all_goals()` 取消正在执行的轨迹目标，控制器立即停在当前位置。

**使用**：
```bash
# 终端1：启动旋转
rosrun fans_move fans_move.py

# 终端2：瞬间停止
rosrun fans_move fans_stop.py
```

---

## 操作 11：验证

### 验证 1：检查包是否被 ROS 找到

```bash
source /home/lste/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/lste/PX4_Firmware:/home/lste/PX4_Firmware/Tools/sitl_gazebo

rospack find wind_generator_description   # 应输出路径
rospack find mavlink_sitl_gazebo          # 应输出路径
rospack find px4                          # 应输出路径
```

### 验证 2：检查 launch 文件语法

```bash
roslaunch px4 wind_generator_demo.launch --ros-args
# 应输出参数列表，无报错
```

### 验证 3：启动风力发电机 (不带无人机)

```bash
roslaunch fans_moveit_config gazebo.launch
```
Gazebo 里应看到风力发电机模型。

### 验证 4：旋转风扇

```bash
rosrun fans_move fans_move.py
```
Gazebo 里叶片应逆时针旋转。

### 验证 5：启动风力发电机 + 无人机

```bash
roslaunch px4 wind_generator_demo.launch spawn_drone:=true
```

三个终端分别运行：
```bash
# 终端 A: 通讯桥梁
cd /home/lste/XTDrone/communication && python multirotor_communication.py iris 0

# 终端 B: 风扇旋转
rosrun fans_move fans_move.py

# 终端 C: 键盘控制
cd /home/lste/XTDrone/control/keyboard && python multirotor_keyboard_control.py iris 1 vel
```

---

## 修改文件汇总

| 序号 | 文件 | 操作 | 说明 |
|------|------|------|------|
| 1 | `fans_moveit_config/launch/planning_context.launch` | 编辑 | `command="xacro"` → `textfile` |
| 2 | `/home/lste/catkin_ws/src/` | 新建软链接 ×4 | 链接 4 个 ROS 包到工作空间 |
| 3 | `PX4_Firmware/Tools/sitl_gazebo/worlds/wind_farm.world` | 新建 | 200×200m 平面 + 光照 + 物理 |
| 4 | `PX4_Firmware/launch/wind_generator_demo.launch` | 新建 | XTDrone 集成启动 |
| 5 | 系统包 | apt install | 7 个 ROS 依赖 |
| 6 | `fans_move/scripts/fans_move.py` | chmod +x | 添加执行权限 |
| 7 | `wind_generator_description/urdf/wind_generator.xacro` | 编辑 | 关节轴 `0 0 -1` → `0 0 1` (翻转旋转方向) |
| 7b | `fans_move/scripts/fans_move.py` | 编辑 | 轨迹目标改回正数，与新轴一致 |
| 8 | `PX4_Firmware/launch/wind_generator_demo.launch` | 编辑 | `ns="uav0"` → `ns="iris_0"` |
| 9 | `fans_moveit_config/launch/gazebo.launch` | 重写 | 改用 wind_farm.world |
| 10 | `fans_move/scripts/fans_stop.py` | 新建 | 瞬间停止叶片旋转 |
