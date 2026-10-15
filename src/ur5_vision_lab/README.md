# 第一课：两小时完成相机 → 识别 → 深度 → TF → RViz

适用：Ubuntu 20.04 / ROS 1 Noetic / Gazebo Classic。今日成果是视觉定位链路，尚不包含夹爪和抓放动作。

工程可以向现有 ROS Gazebo 世界添加桌面、RGB-D 相机和方块，也可以集中启动 UR5 场景。无需修改系统安装的 UR5 文件。UR5 在本实验中不接受运动命令；先理解视觉数据链路，再接运动任务。

## 今日四个完成标准

1. 在 Ubuntu VS Code 中打开、构建 catkin 包。
2. RViz 能显示相机画面和带检测框的图像。
3. 两个 Python 节点分别完成图像定位和 TF 世界坐标转换。
4. 改变方块位置，世界坐标和绿色估计模型随之变化。

## 0～20 分钟：准备与构建

### 安装依赖

在 Ubuntu 执行：

```bash
sudo apt -o Acquire::ForceIPv4=true install unzip python3-opencv python3-numpy ros-noetic-cv-bridge ros-noetic-message-filters ros-noetic-tf2-geometry-msgs ros-noetic-gazebo-plugins
```

集中启动 UR5 的入口需要已经安装 ros-noetic-universal-robots，与你此前的 UR5 环境一致。

### 放入工作空间

把工程 ZIP 复制到 Ubuntu 主目录，文件名保持 `ur5_vision_lab_lesson01.zip`。执行：

```bash
mkdir -p ~/catkin_ws/src
unzip ~/ur5_vision_lab_lesson01.zip -d ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack find ur5_vision_lab
```

若已存在同名包，先保留自己的修改；不要重复解压覆盖自己已经编辑的文件。源文件应位于 `~/catkin_ws/src/ur5_vision_lab/`，不要多嵌套一层同名目录。

### 打开 VS Code

```bash
code ~/catkin_ws
```

如果没有 code 命令，直接在 VS Code 中“打开文件夹”，选择 catkin_ws。选择 Python 解释器 `/usr/bin/python3`。每个新终端 source Noetic 与工作空间；编辑器选择解释器不等于加载 ROS 环境。

`vscode-example` 包含可选的 `.vscode/settings.json` 与 tasks.json 示例。放到工作空间根目录的 `.vscode` 中即可使用；若已存在配置，合并需要的条目。构建快捷键 Ctrl+Shift+B 调用 catkin_make。它只是编辑/构建配置，不会自动启动仿真。

## 20～45 分钟：先看到相机画面

### 方式 A：完整启动（推荐建立本课基线）

在以前启动 Gazebo、MoveIt、RViz 和实验脚本的终端中分别按 Ctrl+C，结束上一轮实例，然后在新终端执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur5_vision_lab demo.launch
```

这个命令启动 UR5 Gazebo、桌面、相机、方块、两个处理节点和 RViz。本课不启动 MoveIt，不需要执行机械臂轨迹。

若尚未安装 UR5 包，可以先执行 `roslaunch ur5_vision_lab demo.launch with_ur5:=false` 验证相机链路。在 RViz 中取消 UR5 显示项，其他显示仍适用。

### 方式 B：添加到已经运行的 UR5 Gazebo

若现有 Gazebo 已正常运行，保留它，只启动：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch ur5_vision_lab sensors.launch
```

这种方式不启动另一个 Gazebo。现有 RViz 手动添加：

- Image：`/camera/color/image_raw`。
- Image：`/vision_lab/debug_image`。
- Marker：`/vision_lab/scene_marker`。
- MarkerArray：`/vision_lab/markers`。
- Fixed Frame：`world`。

方式 A 与 B 选一种。已经存在本课模型时，场景节点会复用；改动 SDF、相机位置或安装参数后，需要重新启动该实验的 Gazebo，保证真实相机与 TF 一致。

### 检查主题

另开终端，source 环境后逐条执行：

```bash
rostopic list
rostopic hz /camera/color/image_raw
rostopic echo -n 1 /camera/color/camera_info
rostopic echo -n 1 /vision_lab/perception_status
```

hz 是连续命令，观察后按 Ctrl+C 退出。不要对完整 image_raw 做无条件 echo，会输出大量像素数据。

应看到的内容：Gazebo 中有灰色工作台和红色方块；RViz Image 面板显示俯视图；Debug 面板的方块外有绿框。帧率以实际测量为准，传感器目标为 10 Hz。

## 45～75 分钟：理解节点、像素和坐标系

查看：

```bash
rosnode info /vision_lab/perception
rosnode info /vision_lab/localization
rostopic echo -n 1 /vision_lab/object_surface_camera
rostopic echo -n 1 /vision_lab/object_center_world
rosrun tf tf_echo world vision_lab_camera_optical_frame
```

数据流：

```text
Gazebo 相机
  /camera/color/image_raw
  /camera/depth/image_raw
  /camera/color/camera_info
           ↓
  /vision_lab/perception
           ↓ PointStamped：表面点，光学坐标系
  /vision_lab/object_surface_camera
           ↓ TF
  /vision_lab/localization
           ↓ PointStamped / PoseStamped / MarkerArray
  世界中的表面点、已知方块中心、绿色估计模型
```

在 VS Code 阅读这几个文件：

| 文件 | 本课重点 |
|---|---|
| scripts/perception_node.py | 订阅、回调、RGB/深度同步、发布结果 |
| src/ur5_vision_lab/vision_math.py | 红色分割、深度单位、三维反投影 |
| scripts/localization_node.py | TF 转换、PoseStamped、MarkerArray |
| launch/sensors.launch | 参数、命名空间、静态相机坐标系 |
| models/camera.sdf | 相机与 ROS 传感器插件 |

理解公式：X=(u-cx)Z/fx、Y=(v-cy)Z/fy。这里 Z 是光轴深度，单位米，不是世界高度，也不是任意方向的欧氏距离。

相机像素向右对应光学 X 正向、向下对应光学 Y 正向，光学 Z 朝镜头前方。本例相机向下看，因此方块上表面在光学坐标系的 Z 约为 0.96 m，在 world 中的 z 约为 0.24 m。

默认方块中心在 world 中约为 `(0.45, 0.30, 0.22)` m。相机实际观测上表面；程序根据本例已知 4 cm 方块高度，减去 2 cm 得到中心。这不是对任意物体的通用尺寸/姿态识别。

定位节点不读取 Gazebo 的物体坐标。对象移动服务仅用于测试输入。

## 75～100 分钟：亲自改变一个变量

### 练习 A：移动方块

```bash
rosrun ur5_vision_lab move_cube.py --x 0.55 --y 0.30
```

等待方块落稳后，查询：

```bash
rostopic echo -n 1 /vision_lab/object_center_world
```

预测：世界 x 增加约 0.10 m；画面里的移动方向不一定与你对世界坐标的直觉相同。再移动 y：

```bash
rosrun ur5_vision_lab move_cube.py --x 0.55 --y 0.40
```

检查 Debug 框、世界坐标与绿色估计模型都随之变化。可用桌面范围已在辅助脚本里限制，避免把物体放到桌外。恢复：

```bash
rosrun ur5_vision_lab move_cube.py --x 0.45 --y 0.30
```

### 练习 B：理解参数与检测失效

```bash
rosparam set /vision_lab/perception/min_area 100000
rostopic echo -n 1 /vision_lab/detected
```

人为把最小轮廓面积设得太大，应看到检测失效、绿色估计模型约 0.6 秒后过期。恢复：

```bash
rosparam set /vision_lab/perception/min_area 60
```

程序每帧读取该参数，不需要为这个练习重启节点。它让你看到“相机有图像”与“算法成功检测”是两个不同状态。

## 100～120 分钟：记录成果与扩展

### 记录一段定位数据

```bash
rosbag record -O ~/vision_position_$(date +%Y%m%d_%H%M%S).bag /vision_lab/object_center_world /vision_lab/detected /tf /tf_static
```

录制期间移动方块两次，之后 Ctrl+C 结束。该记录包含定位与坐标变换，不包含完整 RGB-D 图像。用 `rosbag info 文件路径` 查看录到哪些主题。

### 扩展任务（核心链路完成后再做）

1. 修改 cube.sdf 的颜色，重新启动场景，观察红色检测是否失败；再修改检测阈值支持新颜色。
2. 修改相机高度，通过 sensors.launch 的 camera_z 参数同时改变相机安装与 TF；重新启动整个场景，观察图像尺寸与光学深度变化，但世界定位应保持接近。
3. 绘制三组方块位置的预测/测量对照表。
4. 尝试读取点云 `/camera/depth/points`，解释它与深度图的关系。

此时还不让机械臂向检测点运动：先检查坐标、工具偏移、可达范围和碰撞场景，下一课再接夹爪与任务流程。

## 结束时提交给自己的成果

- Gazebo 场景截图。
- 带检测框的图像截图。
- 两个节点的输入/输出主题表。
- 三组物体位置及测量结果。
- 一份 ROS bag 或实验记录。
- 解释 source、Publisher、Subscriber、CameraInfo、PointStamped、TF 各自的作用。

## 常见故障的检查顺序

- 找不到包：确认目录位于 catkin_ws/src，catkin_make 成功，当前终端 source devel/setup.bash。
- Python 导入错误：使用系统 Python，确认依赖安装；不要用未配置 ROS 的另一套环境启动。
- 相机主题不存在：看 scene 节点是否成功生成 camera，以及 Gazebo 是否报告插件加载失败。
- 相机主题存在但不出帧：确认 Gazebo 没有暂停，查看传感器/渲染相关日志。
- 图像有、定位没有：查询 perception_status；检查颜色、有效深度和 CameraInfo。
- 相机点有、世界点没有：查询 localization_status，并用 tf_echo 检查相机坐标关系。
- RViz 没有绿色模型：检查 MarkerArray Topic、Fixed Frame world，以及是否持续检测成功。
- 新相机位置和 TF 不一致：关闭实验 Gazebo后重新启动，避免复用旧相机模型。
- without UR5 模式下 RobotModel 报错：取消该显示项，它不影响相机练习。

## 验证范围

工程生成端是 Windows，无法直接运行你的 ROS/Gazebo。生成端可检查 Python 语法、XML/JSON、深度单位、反投影与相机坐标几何；OpenCV 检测和 Gazebo 传感器需要在 Ubuntu 中实际验证。

Ubuntu 安装依赖后可执行：

```bash
python3 ~/catkin_ws/src/ur5_vision_lab/tests/test_vision_math.py
```

相机插件配置根据 Noetic 官方源码核对：
https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp

UR5 启动复用官方入口：
https://github.com/ros-industrial/universal_robot/blob/noetic-devel/ur_gazebo/launch/ur5_bringup.launch
