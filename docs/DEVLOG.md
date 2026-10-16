# UR5 视觉拾取仿真开发日志

## 001：建立 Git 版本记录

### 目标
将现有 ROS 学习代码纳入版本管理，为后续开发建立起点。

### 当前环境
- Ubuntu 20.04
- ROS Noetic
- 在 Ubuntu 虚拟机中使用 VS Code 开发

### 已完成
- 创建工作区 Git 仓库。
- 记录 ur5_vision_lab 的现有代码。
- 排除编译目录、Python 缓存和本地编辑器配置。
- 创建基线提交：14190fc。

### 验证
运行 git log -1 --oneline，可以看到基线提交。
运行 git status，显示工作区干净。

### 本次遇到的问题
1. 最初将 .gitignore 建在 .vscode 内，根目录忽略规则没有生效。
   解决：在 catkin_ws 根目录创建 .gitignore。
2. 在家目录执行 git commit，提示不是 Git 仓库。
   解决：先进入 ~/catkin_ws，再执行 Git 命令。

### 下一步
逐步搭建虚拟吸盘拾取实验，并为每个实验记录代码变化和实际结果。

## 002:将节点之间的关系确定
- 比如：study_scene.launch
         → 已有的 demo.launch
               → UR5 的 Gazebo 仿真
               → 场景、摄像头和视觉节点
### 修改
- 创建 ur5_suction_lab 包。
- 在 package.xml 中声明 ur5_vision_lab 运行依赖。
- 添加 study_scene.launch，包含已有的 demo.launch。
- 修改工作区 .gitignore，将新包纳入版本管理。

### 验证命令
```bash
roslaunch ur5_suction_lab study_scene.launch
rostopic hz /camera/color/image_raw
```

### 实际结果
- 摄像头图像话题持续发布，测得约 10 Hz，使用仿真时间。
- Gazebo 中的场景显示情况：填写自己看到的模型。
