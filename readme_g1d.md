# G1-D 单臂视觉接入方案

## 目标和范围

当前迁移目标限定为 **一侧手臂的 7 个关节 + 同侧 Dex1 夹爪**。躯干、头部、底盘和另一侧手臂不作为控制目标。其他关节状态仍可作为只读机器人状态提供给 MoveIt 的环境模型。

建议技术分工：

```text
Show-Harness：任务流程、VLM API 调用、用户指令
相机 ROS 2 话题：左右 RGB、CameraInfo
本机双目几何：左右图像校正、视差/深度、物体三维位置
G1-D URDF/SRDF + MoveIt 2：选定手臂的逆解、碰撞检查和关节轨迹
已有 Unitree SDK2 ROS 2 桥：读取关节状态、执行轨迹、控制所选夹爪
```

MoveIt 2 是本项目第一阶段的规划核心。师兄当前配置已经使用 KDL 求解单臂运动学；Pinocchio 不是启动本方案的前置依赖。第一版不要让 VLM 直接生成关节角或自由格式距离，也不要让 Harness 另起 Unitree LowCmd 发布者。

## 当前代码状态

本次新增的代码均位于本仓库：

- `core/g1d/single_arm.py`：`left`/`right` 单臂配置，提供 MoveIt group、7 个关节、对应夹爪 Action 名称；轨迹校验拒绝另一侧或双臂关节集合。
- `core/g1d/vision_geometry.py`：VLM JSON 检查、双目去畸变/校正和 OpenCV SGBM 视差计算、视差转米制深度、目标框内稳健深度估计、像素反投影、相机到基座的坐标变换、夹爪工具坐标轴小步平移。
- `core/g1d/vision_api.py`：相机图片 → 大模型 → JSON 结果的 API 层。两种 transport：`openai_compat` 复用仓库现有 `core/vlm/vlm_client.py`（provider 方言、限流重试、密钥刷新全部继承）；`custom` 按字段模板 POST 到任意 HTTP 接口并按点号路径取回结果。负责图片编码、prompt、JSON 校验和"目标不可见"的约定。
- `core/g1d/camera_source.py`：订阅相机桥话题并给出**新鲜** RGB 帧。桥发布的是 `bgr8`，这里统一转成 RGB；每帧检查时效，冻结画面会报错而不是给出一个看似合理的检测结果。ROS 导入是惰性的，离线环境导入本模块不报错。
- `scripts/g1d/vision_detect.py`：视觉 API 的命令行入口。取一帧 → 调 API → 打印一行 JSON。**不做深度、不读 TF、不发机器人命令**，输出里 `action_sent` 恒为 `false`。
- `configs/robot_g1d.yaml`：G1-D 配置。`vision_api` 段描述 API 端点；`vlm` / `vlm_backends` 段沿用全仓库统一的凭据与模型选择方式。
- `tests/test_g1d_vision_api.py`：视觉 API 层测试（不需要网络、密钥或 ROS）。
- `scripts/g1d/inspect_target.py`：按固定最大频率订阅新鲜 RGB 图并调用仓库现有 VLM API；提供标定文件时检查左右同步时间戳、CameraInfo，并输出深度和左相机光学坐标。它不会发送机器人命令。
- `scripts/g1d/pose_goal.py`：向现有 `/plan_kinematic_path` 请求单臂末端位姿规划；规划为默认模式；只有加 `--execute` 并在终端输入 `YES` 才请求 `/execute_trajectory`。脚本不会发布 Unitree LowCmd。
- `scripts/g1d/gripper_goal.py`：只请求显式选择的一侧夹爪 Action；只有加 `--send` 并输入 `YES` 才发送。开口单位为米。
- `tests/test_g1d_vision_geometry.py`：不连接 ROS 或真机的数学、输入格式和单臂选择测试。
- `docs/G1D_FIELD_CHECKLIST_2026-10-25.md`：2026-10-25 现场操作与信息采集步骤。

快速检查：

```bash
cd /home/yushu/harness_lzh
python3 -m pytest -q tests/test_g1d_interface.py tests/test_g1d_vision_geometry.py
python3 -m pytest -q tests/test_g1d_vision_api.py
python3 scripts/g1d/pose_goal.py --help
python3 scripts/g1d/gripper_goal.py --help
python3 scripts/g1d/inspect_target.py --help
python3 scripts/g1d/vision_detect.py --help
```

## 尚未接通的部分

当前代码是可检查的接入骨架，不代表 RGB-D 到真机运动已经完成：

1. 相机桥发布左右图像和 `CameraInfo`，不发布深度图。新增的 `StereoDepthProcessor` 可以在**提供有效、实测、与当前图像分辨率匹配的标定文件后**生成深度；当前还没有 G1-D 相机的已验证标定文件。
2. 相机标定、双目同步和基线数据必须现场核验。桥的时间戳是接收时间；代码不推断相机到机器人的 TF。
3. 相机到 `torso_link` 的外参必须测量或标定；如果相机随头部运动，需要动态 TF。当前不能用未经确认的变换生成实机目标。
4. URDF/SRDF 当前规划末端是 `*_wrist_yaw_link`，不是已标定的夹爪 TCP。`pose_goal.py --tip-link` 必须给出 URDF 中真实存在、且 MoveIt 可规划的链接。先用腕部链接只能检查规划链，不能声称到达了夹爪尖端。
5. 当前 SDK Action 服务端同时提供左右手臂和左右夹爪接口。Harness 的 `SingleArmProfile` 和客户端只发选定侧命令，但**服务端目前没有选边白名单**；开全局实机开关后，另一个侧的 Action 也仍可接受命令。需要在实际运行的 SDK 桥内落实侧向白名单后，才算底层只开放一侧。不能同时运行两个 Unitree LowCmd 写入者。
6. 视觉 API 层（相机 → 大模型 → JSON）已经接通并可独立运行，见上文"视觉 API 层"一节；但检测结果还没有自动送给 `pose_goal.py`，相机到基座的变换也只在显式提供并验证 `--base-frame` 时读取 TF。整个视觉流程不会自动触发运动。
7. Dex1 电机角度到实际夹爪开口的关系仍需实机校准。现有 0–0.09 m 是接口名义范围，不是已完成的夹爪宽度/力标定。

## VLM 与 RGB-D 数据约定

VLM API 第一阶段只负责从 RGB 图像中返回目标类别和归一化图像框。约定 JSON：

```json
{
  "label": "cup",
  "bbox_norm": [0.25, 0.20, 0.70, 0.85],
  "confidence": 0.92
}
```

`bbox_norm` 顺序为 `[x_min, y_min, x_max, y_max]`，范围为 `[0,1]`。VLM 响应不包含机械臂关节角、自由格式移动距离或可直接执行的 ROS 命令。程序按图像框从本机深度图取有效深度中位数，再用相机内参反投影：

```text
Z = f * B / disparity
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

其中 `f` 以像素计，`B` 以米计，disparity 以像素计。深度图要求单位为米、左右图完成校正、并与用于检测的 RGB 图逐像素对齐。`T_base_camera` 还必须来自经确认的 TF/外参。`detection_to_base_point()` 输出的是目标点，不是抓取位姿；接近方向、夹爪朝向和 TCP 仍由本机任务规划逻辑确定。

相机可以保持固定帧率发布，但不建议每帧都调用 VLM API。先对最新图像做按需识别；运动由本机规划、执行并重新观察。

## 视觉 API 层（相机 → 大模型 → JSON）

这条链路的唯一职责是：**取一帧 RGB → 调大模型 → 产出一行 JSON**。它不读深度、不读 TF、不发机器人命令，因此可以脱离实机单独跑通和验证。

### 代码位置

| 文件 | 职责 |
| --- | --- |
| `core/g1d/camera_source.py` | 订阅相机桥话题 → 新鲜 RGB 帧（`bgr8` 转 RGB，带时效检查） |
| `core/g1d/vision_api.py` | 图片编码、prompt、JSON 校验、两种 transport |
| `scripts/g1d/vision_detect.py` | 命令行入口，每行打印一条 JSON 结果 |
| `configs/robot_g1d.yaml` | 端点、模型、阈值配置 |

### 两种 transport

**`openai_compat`（默认）** 直接复用仓库现有的 `core/vlm/vlm_client.py`。所以 provider 方言（vLLM 的 guided decoding、OpenAI 的 `max_completion_tokens`、Gemini 的 `reasoning_effort`）、429 限流重试、密钥刷新全部继承，不需要在 G1-D 侧重写一遍。端点和模型用全仓库统一的 `vlm` / `vlm_backends` 配置。

**`custom`** 给不保证说 OpenAI 方言的网关用。请求体按字段模板渲染，响应按点号路径取值：

```yaml
vision_api:
  transport: custom
  base_url: https://gateway.example/v1/detect
  method: POST
  headers:
    X-Tenant: g1d
  request_template:
    model: "{model}"
    image: "{image_base64}"     # 或 "{image_data_url}"（带 data:image/png;base64, 前缀）
    prompt: "{prompt}"
  response_path: "data.result"  # 数字段可索引列表："data.detections.0"
```

模板可用占位符：`{model}`、`{image_base64}`、`{image_data_url}`、`{prompt}`。

### 跑起来

```bash
cd /home/yushu/harness_lzh
source /opt/ros/humble/setup.bash

# 相机话题 → API（实时）
python3 scripts/g1d/vision_detect.py \
  --topic /g1d/cameras/head_left/image_raw \
  --target "steel bearing"

# 静态图片 → API（离线调试，不需要 ROS）
python3 scripts/g1d/vision_detect.py --image /tmp/frame.png --target "steel bearing"
```

输出（一行一条 JSON）：

```json
{"found": true, "label": "steel bearing", "confidence": 0.88,
 "bbox_norm": [0.31, 0.42, 0.55, 0.61], "image_size": [640, 480],
 "model": "gw-detect-v1", "transport": "custom", "vlm_latency_s": 0.041,
 "action_sent": false}
```

### 约定

- **"没看见"是正常结果，不是错误。** 空 `label` 或 0 置信度 → `found: false`，`bbox_norm` 为 `null`。只有传输层故障（网络、HTTP 状态）才抛异常。
- **低于 `min_confidence` 阈值 → `found: false`**，不作为低置信度命中返回。阈值在 `vision_api.min_confidence` 配置。
- **框的坐标是归一化的、相对整幅输入图的**。`image_size` 报告的是**模型实际看到的尺寸**（设了 `max_image_side` 会先缩放）。
- **响应里不含关节角、不含自由格式距离、不含可执行 ROS 命令。** 感知和运动是解耦的。
- `action_sent` 恒为 `false`。这个脚本永远不碰机器人。

双目标定文件的字段如下，数值必须使用实验室实测结果，`T_right_left_m` 单位是米。不要把下方零值填入实际文件：

```yaml
image_width: <单个左/右图像宽度像素>
image_height: <单个左/右图像高度像素>
K_left: [[...], [...], [...]]
D_left: [...]
K_right: [[...], [...], [...]]
D_right: [...]
R_right_left: [[...], [...], [...]]
T_right_left_m: [[...], [...], [...]]
```

现场拍完标定板后，还需离线计算 `R_right_left` 和 `T_right_left_m` 并复核深度误差，才能使用这个文件。当前没有自动标定程序，也没有真机标定数据。

## 明天（2026-10-25）现场方案

按下面的顺序做，先只读检查和 dry-run。详细终端步骤、记录表和停止条件见[明天现场清单](docs/G1D_FIELD_CHECKLIST_2026-10-25.md)。

### 1. 确认运行版本和所选侧

确定测试左臂还是右臂，记录当前运行的 `g1d_ros2_ws` 版本/提交，并确认现场没有另一进程向 Unitree SDK 发布命令。先不要同时启动其他 SDK 控制器。

### 2. 检查相机、标定和数据新鲜度

```bash
source /opt/ros/humble/setup.bash
source /home/yushu/g1d_ros2_ws/install/setup.bash
ros2 launch g1d_camera_bridge cameras.launch.py show_viewer:=true publish_rate:=15.0
```

另一终端检查：

```bash
ros2 topic hz /g1d/cameras/head_left/image_raw
ros2 topic hz /g1d/cameras/head_right/image_raw
ros2 topic echo /g1d/cameras/head_left/camera_info --once
ros2 topic echo /g1d/cameras/head_right/camera_info --once
ls -l ~/.ros/g1d_camera_calibration/
```

确认左右相机确实同步采集、图像无冻结、`CameraInfo` 含有效内参且尺寸与图像一致。若没有双目标定/基线，不要把图像识别结果解释成可靠的米制深度。

如已装好本仓库运行环境，可先对 RGB 做一次低频 VLM 识别（每次约间隔 5 秒；只调用 API，不碰手臂）：

```bash
cd /home/yushu/harness_lzh
python3 scripts/g1d/inspect_target.py \
  --config configs/robot_piper.yaml \
  --topic /g1d/cameras/head_left/image_raw \
  --target "桌面上的红色杯子" \
  --count 1
```

`configs/robot_piper.yaml` 在这里只提供 VLM provider/model/API 配置，不会连接 Piper。API 凭据从 `configs/secrets.env` 或 `configs/secrets.local.env` 读取。若尚无密钥或网络不可用，先用相机 viewer 验证图像，不影响真机 dry-run。

**没有标定文件时不要传 `--stereo-calibration`。** 若已由实验数据生成并核对了 G1-D 专用标定文件，可以增加参数计算深度：

```bash
python3 scripts/g1d/inspect_target.py \
  --config configs/robot_piper.yaml \
  --topic /g1d/cameras/head_left/image_raw \
  --right-topic /g1d/cameras/head_right/image_raw \
  --target "桌面上的红色杯子" \
  --stereo-calibration /path/to/verified_g1d_stereo.yaml \
  --count 1
```

默认输出的 3D 点在校正后左相机光学坐标系。只有当相机对应 TF 已实际验证时，才添加 `--base-frame torso_link` 请求同一图像时间戳的坐标变换。该图像时间戳仍是桥接接收时间，不是曝光时间；相机随头部运动时，先核实动态 TF 与时间同步。

### 3. 检查单臂 MoveIt 状态并只做规划

```bash
source /opt/ros/humble/setup.bash
source /home/yushu/g1d_ros2_ws/install/setup.bash
ros2 launch g1d_bringup real_moveit.launch.py
```

不传 `real_execution:=true` 或 `gripper_real_execution:=true` 时，按当前 launch 默认值运行 dry-run。确认 `/joint_states` 和 Action：

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /dex1/motor_states --once
ros2 action list -t
```

在仓库根目录使用 MoveIt 位姿客户端做单臂、最多 1 cm 的规划示例；将侧和链接名改成现场确认值：

```bash
cd /home/yushu/harness_lzh
python3 scripts/g1d/pose_goal.py \
  --side left \
  --base-frame torso_link \
  --tip-link left_wrist_yaw_link \
  --step-tool-x 0.005
```

这条命令**只规划**，然后在 RViz 查看 `/display_planned_path`。当前 `left_wrist_yaw_link` 是腕部模型链接，并非已标定 TCP。若 URDF 中有确认过的 TCP，应使用 TCP 链接名；否则只把这一步当作腕部规划链通断检查。

夹爪可以先做 Action 可用性检查，不发送动作：

```bash
python3 scripts/g1d/gripper_goal.py --side left --opening-m 0.04
```

只有在夹爪周围无障碍、反馈正常且 launch 明确处于 dry-run 时，才可评估 Action 路径。真机动作还需要夹爪侧向白名单和现场负责人确认。

### 4. 现场记录并反馈

记录：选定侧、关节反馈、左右图像频率、图像大小、两侧 `CameraInfo` 内容、标定文件是否存在、相机固定/转动情况、有效的末端 TCP 链接、夹爪当前原始反馈和 SDK 桥实机开关状态。把这些结果填入现场清单。确认侧向白名单没有落实之前，不要开启全局真机控制开关。

## 六项接通后的下一阶段

六项缺口解决后，只能先实现一个**受限的视觉引导接近演示**：识别物体 → 深度定位 → 坐标转换 → 单臂 MoveIt 规划 → 人工确认 → 小步执行。之后还要继续：

1. 将目标物体和桌面/周边障碍物写入 MoveIt planning scene，验证模型碰撞几何与真实夹爪一致。
2. 建立任务状态机：观察、检测、估距、预接近、重新观察、夹取、抬起、验证抓取、退出；每一步失败都停止并保留状态。
3. 在空载、已知目标和多个工作位置下重复测试，记录目标点误差、规划成功率、动作完成状态和夹爪反馈。
4. 用真实抓取成功率和滑落/空抓反馈校准夹爪宽度与抓取策略。
5. 最后再决定是否增加连续视觉伺服或 Pinocchio 快速运动学算法；第一版仍保持 VLM 低频识别、本机运动规划与反馈闭环。

六项接通不等于“可以无人监督抓取”。它只说明视觉目标可以进入受限的单臂规划链。
