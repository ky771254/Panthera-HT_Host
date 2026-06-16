# Panthera-HT Host

![Panthera-HT Host UI](images/1.png)

这是 Panthera-HT 六轴机械臂的上位机项目，包含真机控制 SDK 示例、数字孪生后端和 Web 前端。项目主要用于：

- 连接 Panthera-HT 真机并进行位置、重力补偿、阻抗等控制
- 在浏览器中实时显示机械臂 3D 状态、关节状态和末端状态
- 从 Web 页面运行 `panthera_python/scripts/` 下的 SDK 示例脚本
- 在没有真机时使用 Demo 仿真模式调试前后端界面

## 目录结构

```text
.
├── install.sh                         # Ubuntu + conda 一键安装脚本
├── backend.sh                         # 一键启动后端，默认真机模式
├── frontend.sh                        # 一键启动前端
├── Panthera_digital_twin-main/
│   ├── backend/                       # Flask + Socket.IO 后端
│   │   ├── app.py                     # 主后端入口和控制逻辑
│   │   ├── panthera_sim.py            # Demo 仿真机器人
│   │   └── run_script.py              # Web 端脚本运行器
│   ├── frontend/                      # Three.js + Vite Web 前端
│   ├── robot_param/                   # 机器人参数配置
│   └── Panthera-HT_description/       # URDF 和模型资源
└── panthera_python/
    ├── scripts/                       # SDK 示例脚本，Web Scripts 面板会读取这里的 .py 文件
    ├── motor_whl/                     # hightorque_robot SDK wheel
    └── requirements.txt               # Panthera 高层库 Python 依赖
```

## 运行环境

推荐：

- Ubuntu
- 已安装 Miniconda 或 Anaconda
- Python 3.10 conda 环境
- 浏览器访问前端页面
- 真机模式需要连接 Panthera-HT 机械臂和对应串口/CAN 设备

系统依赖由 `install.sh` 安装：

- `build-essential`
- `cmake`
- `git`
- `libserialport-dev`
- `udev`

默认不安装 `libyaml-cpp.so.0.6 / yaml-cpp 0.6.1`。如确实需要，可手动运行：

```bash
INSTALL_YAML_CPP_06=1 ./install.sh
```

## 一键安装

在只安装了 conda 的 Ubuntu 电脑上，运行：

```bash
chmod +x install.sh backend.sh frontend.sh
./install.sh
```

脚本会创建 conda 环境：

```text
Panthera_host
```

并安装：

- 后端 Python 依赖
- `panthera_python` 高层库依赖
- `hightorque_robot` wheel
- `pynput`
- 前端 Node/npm 依赖
- 串口 udev 权限规则

如果只想安装 conda/Python/npm 依赖，不想改系统依赖：

```bash
INSTALL_SYSTEM_DEPS=0 ./install.sh
```

删除 conda 环境：

```bash
conda env remove -n Panthera_host
```

## 启动项目

需要分别启动后端和前端，建议开两个终端。

### 1. 启动后端

默认是真机模式：

```bash
./backend.sh
```

等价于：

```bash
cd Panthera_digital_twin-main/backend
conda run --no-capture-output -n Panthera_host python app.py --config ../robot_param/Follower.yaml --port 5000
```

Demo 仿真模式，不连接真机：

```bash
./backend.sh --demo
```

指定配置文件或端口：

```bash
./backend.sh --live --config ../robot_param/Follower.yaml --port 5000
```

### 2. 启动前端

```bash
./frontend.sh
```

默认地址：

```text
http://localhost:3000
```

指定端口：

```bash
./frontend.sh --port 3001
```

## 默认真机模式在哪里改

后端脚本默认真机启动的位置：

```text
backend.sh
```

关键参数：

```bash
MODE="live"
DEFAULT_CONFIG="../robot_param/Follower.yaml"
PORT="5000"
ENV_NAME="Panthera_host"
```

如果想让脚本默认 Demo 启动，可以把：

```bash
MODE="live"
```

改成：

```bash
MODE="demo"
```

前端脚本使用的 conda 环境在：

```text
frontend.sh
```

关键参数：

```bash
ENV_NAME="Panthera_host"
HOST="0.0.0.0"
PORT="3000"
```

## 后端控制逻辑

主要控制逻辑在：

```text
Panthera_digital_twin-main/backend/app.py
```

常用参数位置：

- `CONTROL_FREQ`：控制循环频率，默认 `200 Hz`
- `BROADCAST_FREQ`：WebSocket 状态广播频率，默认 `30 Hz`
- `target_velocity`：Position 模式默认目标速度
- `max_torque`：Position 模式最大力矩
- `control_mode`：默认控制模式
- `gravity_gain` / `tau_limit`：重力补偿相关参数
- `impedance_K` / `impedance_B`：阻抗模式刚度和阻尼
- `impedance_target`：阻抗模式默认目标关节位置

Control Mode 对应关系：

| 模式 | 后端逻辑 | 用途 |
| --- | --- | --- |
| `Position` | 位置控制，调用 `Joint_Pos_Vel` 或仿真状态更新 | 精确发送关节目标位置 |
| `Gravity` | 重力补偿，主要使用 `robot.get_Gravity()` 计算补偿力矩 | 手动拖动、示教 |
| `Impedance` | `K(q_des - q) + B(-dq) + G(q)` | 柔顺控制、外力交互 |

## 前端界面

前端入口：

```text
Panthera_digital_twin-main/frontend/src/main.js
```

主要 UI 文件：

- `src/ui/ConnectionUI.js`：Robot Connection 面板
- `src/ui/JointControlsUI.js`：关节控制面板和 Send Position 逻辑
- `src/ui/ScriptControlUI.js`：SDK Example Scripts 面板
- `src/ui/PanelManager.js`：浮窗拖动、缩放、显示/隐藏
- `index.html`：页面结构和大量样式

当前关节控制逻辑是：拖动关节滑块只更新待发送位置和预览浮窗，点击 `Send Position` 后才发送控制命令。并且当 Control Mode 不是 `Position` 时，关节栏会被灰色遮罩锁定。

## SDK Example Scripts

Web 顶部栏的 `Scripts` 按钮会打开 `SDK Example Scripts` 浮窗。

脚本来源：

```text
panthera_python/scripts/
```

只读取该目录下一层的 `.py` 文件，不包含子目录。运行输出由后端写入日志，再由前端读取显示。

常见脚本包括：

- `0_robot_get_state.py`
- `0_robot_set_zero.py`
- `1_Joint_PD_control.py`
- `1_Joint_PosVel_control.py`
- `2_gravity_compensation_control.py`
- `4_impedance_trajectory_control_with_gra_pd.py`
- `7_keyboard_cartesian_vel_control.py`

真机模式下运行脚本会控制实际硬件，运行前请确认机械臂周围环境安全。

## 机器人配置

默认真机配置：

```text
Panthera_digital_twin-main/robot_param/Follower.yaml
```

配置中会继续引用电机参数和 URDF：

```text
Panthera_digital_twin-main/robot_param/motor_param/
Panthera_digital_twin-main/Panthera-HT_description/
```

如果换机械臂参数，优先从 `robot_param/*.yaml` 和 `robot_param/motor_param/*.yaml` 修改。

## 常用命令

```bash
# 安装
./install.sh

# 后端真机模式
./backend.sh

# 后端 Demo 模式
./backend.sh --demo

# 前端
./frontend.sh

# 前端构建检查
cd Panthera_digital_twin-main/frontend
npm run build

# 删除 conda 环境
conda env remove -n Panthera_host
```

## 注意事项

- `backend.sh` 默认是真机模式，会尝试连接硬件。
- 没有连接真机时，请使用 `./backend.sh --demo`。
- 前端只是界面，实际控制命令由后端 `app.py` 执行。
- `panthera_python/scripts/` 下的脚本可能直接控制真机，运行前请检查脚本内容。
- `.build/` 是旧的 yaml-cpp 源码编译缓存，默认安装流程不需要它。
- 更详细的数字孪生说明见 `Panthera_digital_twin-main/README.md`。
