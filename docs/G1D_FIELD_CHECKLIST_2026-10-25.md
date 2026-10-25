# G1-D 明日现场清单（2026 年 10 月 25 日）

## 目标

验证相机、机器人状态、MoveIt 单臂规划链和夹爪 Action 是否可用，并收集深度与坐标标定所需的信息。默认先做**只读检查和 dry-run**；不要因为 VLM 能识别物体就直接让机械臂追踪图像。

测试范围：选定一侧 7 关节手臂和同侧 Dex1。另一侧及躯干、头部、底盘不作为命令目标。其它关节状态可以只读保留给 MoveIt 机器人状态模型。

## 到现场前准备

- 确认使用左侧还是右侧，并按师兄实际调通情况选择。
- 带一个静止、轮廓清楚、尺寸适中的测试物体；记录它相对相机的大致实测距离。
- 准备双目标定板/棋盘格；若相机没有标定，明天先采集标定图像，不要把 VLM 的像素框转成实机距离。
- 确认急停位置、机械臂运动范围和现场负责人的停止口令。
- 与师兄确认当前 SDK 桥的启动状态和是否有其他程序正在控制 Unitree。整个系统只能有一个 SDK LowCmd 写入者。

## A. 检查相机

### 启动

终端 A：

```bash
source /opt/ros/humble/setup.bash
source /home/yushu/g1d_ros2_ws/install/setup.bash
ros2 launch g1d_camera_bridge cameras.launch.py show_viewer:=true publish_rate:=15.0
```

查看窗口内 head 左右图像是否正常、方向是否符合 `head_order`、是否存在冻结/花屏/左右错位。当前桥默认将 side-by-side 输入拆成左右图像；这本身不能证明曝光同步和双目标定正确。

### 查看数据

终端 B：

```bash
source /opt/ros/humble/setup.bash
source /home/yushu/g1d_ros2_ws/install/setup.bash
ros2 topic hz /g1d/cameras/head_left/image_raw
ros2 topic hz /g1d/cameras/head_right/image_raw
ros2 topic echo /g1d/cameras/head_left/camera_info --once
ros2 topic echo /g1d/cameras/head_right/camera_info --once
ls -l ~/.ros/g1d_camera_calibration/
```

记录左右频率、图像宽高、`K`/`D`/`R`/`P` 矩阵是否有效。相机桥写入的图像时间戳是网络桥收到图像的时间，不是曝光时间；若要做精确双目深度，需确认原始图像在相机端是否同步。

### 深度可用性判断

- 两侧 CameraInfo 空矩阵或校准文件不存在：当前不可算可靠米制深度；只验证 RGB 和 VLM 识别。
- 只有单目内参、没有左右相机相对位姿/基线：仍不足以恢复双目深度。
- 标定图像应覆盖不同距离和倾角，并确保同一时刻拍到左右棋盘格。
- 之后用已知距离的静止物体检查深度量级和重复性。

不要为了明天赶进度临时猜焦距、基线、左右相机偏移或相机到机器人变换。

## B. 检查机器人和 MoveIt（dry-run）

终端 C：

```bash
source /opt/ros/humble/setup.bash
source /home/yushu/g1d_ros2_ws/install/setup.bash
ros2 launch g1d_bringup real_moveit.launch.py
```

这是默认 dry-run 启动方式。保持 launch 默认值，不加 `real_execution:=true` 和 `gripper_real_execution:=true`。

终端 D：

```bash
source /opt/ros/humble/setup.bash
source /home/yushu/g1d_ros2_ws/install/setup.bash
ros2 topic echo /joint_states --once
ros2 topic echo /dex1/motor_states --once
ros2 action list -t
```

确认左右手臂 14 个关节反馈不断更新（只读；命令仍只发选定侧），选定侧关节名称正确，Dex1 反馈在线，MoveIt 的 `/plan_kinematic_path` 和 `/execute_trajectory` 可用，左右夹爪 Action 名称与所选侧匹配。记录网络接口、工作区提交号和所有报错。

## C. 单臂末端位姿规划（只规划，不执行）

从仓库根目录运行。下面用左侧举例；将 `--side`、`--tip-link` 换成实机确认值：

```bash
cd /home/yushu/harness_lzh
python3 scripts/g1d/pose_goal.py \
  --side left \
  --base-frame torso_link \
  --tip-link left_wrist_yaw_link \
  --step-tool-x 0.005
```

预期行为：

1. 读取选定侧 7 个关节的最新状态。
2. 通过 TF 读取 `torso_link` 到目标链接的当前位姿。
3. 生成夹具本地 X 轴方向 5 mm 的目标，程序限制单次平移最多 10 mm。
4. 只向选定手臂规划组请求规划；检查轨迹关节名必须恰好是该侧的 7 个关节。
5. 在 RViz 显示规划路径；不调用执行 Action。

如果提示目标 TF 不存在，说明传入的链接未发布或机器人描述/状态没有起来。不要换成猜测的 TCP 名称；先从实际 URDF/TF 树确认链接。`*_wrist_yaw_link` 是师兄当前 SRDF 规划链的腕部链接，不等于夹爪实际夹取点。

如果规划不可达、轨迹涉及错误关节、路径碰撞或 RViz 起点和实机状态不一致，停止在规划阶段并记录错误。不要尝试把限制调大来“让它通过”。

## D. 检查夹爪 Action（先不发目标）

```bash
cd /home/yushu/harness_lzh
python3 scripts/g1d/gripper_goal.py --side left --opening-m 0.04
```

不传 `--send` 时脚本只显示目标 Action 名称和名义开口，不发送指令。当前桥的夹爪电机角度到开口宽度映射仍是暂定值。现场要先确认空载反馈和机械限位，再和负责人确认是否允许单独做夹爪 dry-run/实机动作。

## E. 真机动作的前置条件

本仓库客户端有选边限制，但师兄当前 C++ SDK 桥的 `real_execution`、`gripper_real_execution` 是全局开关，左右 Action 都已创建。因此“我的脚本只发左侧”不是底层防止右侧 Action 被调用的硬隔离。

**在底层侧向白名单落实、负责人确认当前没有其他 LowCmd 写入者、MoveIt dry-run 路径经过人工检查之前，不开启真机开关。** 用户确认执行时，单臂位姿脚本需要同时带 `--execute` 并在提示时输入 `YES`；夹爪脚本需要 `--send` 并输入 `YES`。这些确认不能代替侧向白名单、急停和现场监护。

## F. 记录并发回的信息

请记录或拍照保存以下信息，后续再填标定值和接 API：

```text
选定侧：left / right
g1d_ros2_ws 分支和提交号：
机械臂关节状态是否持续更新：是 / 否
相机左右图像频率：
左右图像宽高：
左/右 CameraInfo 是否有效：
双目外参/基线是否已有：
两幅图是否硬件同步：
相机相对机器人安装链接：固定 / 随头部转动
相机到 torso_link 外参是否已有：
MoveIt 目标链接：
是否有经过测量的 gripper TCP：
单臂 dry-run 规划是否成功：
夹爪反馈是否在线、当前原始数值：
SDK 桥真实开关当前值：
现场出现的错误信息：
```

## 明天完成后的下一步

1. 若无有效双目标定：先完成棋盘格标定和已知距离深度核验。
2. 若相机坐标外参缺失：固定相机状态并标定相机到 `torso_link` 的变换；若头部会动，配置动态 TF。
3. 核实 URDF/SRDF 末端与实际夹爪，明确 TCP，并测量夹爪开口/电机反馈关系。
4. 在受控环境中实现并验证 SDK 桥单侧白名单，再按同一侧做低速执行测试。
5. 将同步 RGB、深度、CameraInfo、TF 和 VLM JSON 接成一个离线目标定位流程，先显示三维目标点，不发运动命令。
6. 目标点误差和 MoveIt 规划都通过后，才启用“人工确认的小步接近”。抓取、抬升和自动重试属于之后的阶段。
