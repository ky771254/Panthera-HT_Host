# Panthera Digital Twin

A digital twin system for the Panthera-HT 6-DOF robotic arm. The backend connects to the physical robot via the Python SDK or runs in simulation mode. The frontend provides real-time 3D visualization and web-based control using Three.js.

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Project Structure](#project-structure)
- [Environment Setup](#environment-setup)
- [Quick Start](#quick-start)
- [Features](#features)
  - [Connection Panel](#connection-panel)
  - [Control Modes](#control-modes)
  - [Joint Control](#joint-control)
  - [End Effector Panel](#end-effector-panel)
  - [Force/Torque Visualization](#forcetorque-visualization)
  - [Keyboard Control](#keyboard-control)
  - [Waypoints & Trajectory](#waypoints--trajectory)
  - [SDK Script Runner](#sdk-script-runner)
  - [Model & File Management](#model--file-management)
- [Backend API Reference](#backend-api-reference)
- [Robot Configuration](#robot-configuration)
- [Tech Stack](#tech-stack)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Browser (Frontend)                                         │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐  │
│  │ 3D View  │ │ Joint    │ │ Force/   │ │ SDK Script   │  │
│  │ Three.js │ │ Sliders  │ │ Torque   │ │ Runner       │  │
│  └──────────┘ └──────────┘ └──────────┘ └───────────────┘  │
│                       │ WebSocket + REST                    │
└───────────────────────┼─────────────────────────────────────┘
                        │
┌───────────────────────┼─────────────────────────────────────┐
│  Backend (Flask)                                            │
│  ┌────────────────────┼──────────────────────────────────┐  │
│  │  app.py            │                                  │  │
│  │  ├─ REST API       │  /api/move, /api/status ...      │  │
│  │  ├─ WebSocket      │  robot_state (30 Hz broadcast)   │  │
│  │  ├─ Control Loop   │  200 Hz (position/gravity/imped) │  │
│  │  ├─ Force Est.     │  τ_ext → J^T pinv → F_ext        │  │
│  │  └─ Script Runner  │  Whitelisted SDK examples        │  │
│  └────────────────────┼──────────────────────────────────┘  │
│                       │                                      │
│  ┌────────────────────┼──────────────────────────────────┐  │
│  │  Live Mode         │  Simulation Mode (--demo)        │  │
│  │  Panthera SDK ←→   │  PantheraSim (Pinocchio FK/IK)   │  │
│  │  hightorque_robot  │  Pushes joint state via HTTP     │  │
│  │  CAN bus hardware  │  3D view syncs in real time      │  │
│  └────────────────────┴──────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

**Data flow:** Robot/Sim → backend `state_broadcast_loop` (30 Hz) → WebSocket `robot_state` → frontend updates 3D model and UI.

---

## Project Structure

```
Panthera_digital_twin-main/
│
├── backend/                          # Python backend
│   ├── app.py                        # Main entry — REST API + WebSocket + control loop
│   ├── app_cartesian.py              # Cartesian impedance control (standalone)
│   ├── panthera_sim.py               # Virtual Panthera class for simulation mode
│   ├── run_script.py                 # Script launcher (demo subprocess / live thread)
│   └── requirements.txt              # Python dependencies
│
├── frontend/                         # Web frontend
│   ├── index.html                    # Main page + all CSS styles
│   ├── package.json                  # Node dependencies
│   ├── vite.config.js
│   └── src/
│       ├── main.js                   # App entry point — DigitalTwinApp class
│       ├── robot/
│       │   └── RobotConnection.js    # WebSocket client (singleton)
│       ├── ui/
│       │   ├── ConnectionUI.js       # Connection panel + control buttons
│       │   ├── JointControlsUI.js    # Six joint slider panel
│       │   ├── KeyboardControlUI.js  # Keyboard binding + help panel
│       │   ├── ScriptControlUI.js    # SDK script runner panel
│       │   ├── PanelManager.js       # Floating panel layout manager
│       │   └── UIController.js       # Toolbar buttons + theme/language
│       ├── renderer/                 # Three.js scene rendering
│       │   ├── SceneManager.js       # Scene / camera / lighting
│       │   ├── VisualizationManager.js  # URDF model visualization
│       │   ├── CoordinateAxesManager.js # Joint coordinate frames
│       │   ├── HighlightManager.js   # Link/joint highlighting
│       │   ├── EnvironmentManager.js # Background / ground plane
│       │   ├── ConstraintManager.js  # Constraint visualization
│       │   ├── MeasurementManager.js # Distance measurement
│       │   └── InertialVisualization.js
│       ├── adapters/                 # Model format adapters
│       │   ├── URDFAdapter.js        # URDF → UnifiedRobotModel
│       │   ├── MJCFAdapter.js        # MJCF (MuJoCo XML) adapter
│       │   └── USDAdapter.js         # USD format adapter
│       ├── controllers/
│       │   ├── FileHandler.js        # Drag-and-drop file loading
│       │   ├── CodeEditorManager.js  # Code editor integration
│       │   └── MeasurementController.js
│       ├── loaders/
│       │   ├── FileLoader.js         # URL-based file loader
│       │   └── ModelLoaderFactory.js # Model loader factory
│       ├── models/
│       │   └── UnifiedRobotModel.js  # Unified link/joint tree model
│       ├── views/
│       │   └── FileTreeView.js       # Left sidebar file tree
│       └── utils/
│           ├── MathUtils.js
│           ├── DragStateManager.js
│           ├── JointDragControls.js
│           ├── MeshLoader.js
│           ├── FileUtils.js
│           ├── XMLUpdater.js
│           └── i18n.js               # Internationalization
│
├── Panthera-HT_description/          # Panthera-HT URDF + STL meshes
│   ├── urdf/                         # URDF definition files
│   ├── meshes/                       # STL meshes (link1–link6)
│   ├── config/                       # Joint name config
│   └── launch/                       # ROS launch files
│
├── arm_description/                  # Alternative robot arm URDF
│   ├── urdf/
│   ├── meshes/
│   └── launch/
│
├── arm_description_xlb/              # XLB arm URDF
│   ├── urdf/
│   ├── meshes/
│   └── launch/
│
└── robot_param/                      # Robot YAML configuration
    ├── Follower.yaml                 # Follower arm config (default)
    ├── xlb.yaml                      # XLB arm config
    └── motor_param/                  # Motor params (CAN ID, PID, etc.)
        ├── 6dof_Panthera_params_follower.yaml
        └── 6dof_Xlb_params.yaml
```

---

## Environment Setup

### Prerequisites

| Component | Requirement |
|-----------|-------------|
| OS | Linux (x86_64 / aarch64) |
| Python | 3.9+ (conda environment recommended) |
| Node.js | 18+ |
| Package managers | pip + npm |

### 1. Backend Dependencies

```bash
cd backend
pip install -r requirements.txt
```

`requirements.txt` contents:

```
flask>=2.0.0
flask-socketio>=5.0.0
flask-cors>=3.0.0
python-socketio>=5.0.0
pyyaml>=6.0
numpy>=1.20.0
eventlet>=0.33.0
```

To connect to a physical robot, you also need the `hightorque_robot` SDK and `pin` (Pinocchio). See `panthera_python/README.md` for details.

### 2. Frontend Dependencies

```bash
cd frontend
npm install
```

Core packages in `package.json`:

| Package | Purpose |
|---------|---------|
| `three` | 3D rendering engine |
| `urdf-loader` | URDF file loading and parsing |
| `socket.io-client` | Real-time WebSocket communication |
| `d3` | Model structure tree graph |
| `vite` | Dev server and build tool |

---

## Quick Start

### 1. Start the Backend

**Demo / simulation mode (no robot required, recommended for first run):**

```bash
cd backend
python app.py --demo
```

**Live robot mode (requires the panthera conda environment):**

```bash
conda activate panthera
cd backend
python app.py --config ../robot_param/Follower.yaml
```

The backend listens on `http://localhost:5000`. Use `--port` to change the port.

### 2. Start the Frontend

```bash
cd frontend
npm run dev
```

The frontend listens on `http://localhost:3000` with hot reload support.

### 3. Open the Browser

Visit `http://localhost:3000`:

- The page auto-loads the default URDF model from `arm_description/`
- Click **Connect** in the top-right corner to connect to the backend
- All control panels activate once connected

---

## Features

### Connection Panel

Location: top-right floating panel. Expands after clicking Connect.

| Element | Description |
|---------|-------------|
| Status indicator | Green = connected, gray = disconnected |
| Server URL | Backend address, default `http://localhost:5000` |
| Connect / Disconnect | Establish / close WebSocket connection |
| Robot info | Robot name, joint count, mode (Live/Demo) |

### Control Modes

Three modes, switchable via the Control Mode dropdown in the connection panel:

| Mode | Principle | Use Case |
|------|-----------|----------|
| **Position** | Direct position control — `Joint_Pos_Vel(pos, vel, max_torque)` | Precise positioning |
| **Gravity** | Gravity compensation — `τ = G(q)` — zero-force floating | Hand-guiding / teaching |
| **Impedance** | Joint impedance — `τ = K(q_des − q) + B(0 − dq) + G(q)` | Compliant interaction |

**Mode switching behavior:** When switching to Impedance, the target is automatically set to the current joint angles to prevent jumps.

Quick-action buttons:

| Button | Function |
|--------|----------|
| **Home** | Move all joints to zero |
| **Stop** | Hold current position |
| **Set Zero** | Set current encoder position as the zero reference |

### Joint Control

Location: left floating panel. Six joint sliders are auto-generated after loading a URDF model.

- Each joint shows name, current angle, and limit range
- Dragging a slider sends commands to the robot/simulation in real time
- Radian/degree toggle (rad / deg)
- Reset button to return all joints to zero
- **When connected to a live robot:** slider borders turn green, values refresh at 30 Hz from the backend

### End Effector Panel

Location: toggle via the toolbar **End Effector** button.

| Section | Content |
|---------|---------|
| **Position (m)** | End-effector XYZ coordinates |
| **Orientation (deg)** | Roll / Pitch / Yaw (Euler angles) |
| **External Force (N)** | Estimated external force Fx Fy Fz + magnitude (Impedance mode only) |
| **External Torque (Nm)** | Estimated external torque Mx My Mz + magnitude |

Force/torque values use color coding: green (low) → orange (medium) → red (high).

### Force/Torque Visualization

Active only in **Impedance mode**. External forces are estimated from motor torque readings:

```
τ_ext = τ_measured − G(q) − friction(dq)
F_ext = J^T (J J^T + λ²I)^(−1) τ_ext
```

- External force shown as **orange→red 3D arrows** at the end-effector position
- External torque shown as **cyan→blue 3D arrows**
- Arrow direction = force/torque direction, arrow length = magnitude
- Automatically hidden when force < 0.5 N or torque < 0.1 Nm

### Keyboard Control

When connected to a live robot, press keyboard keys to control the arm. Click the toolbar **Keyboard** button to view the key map.

**Position mode:**

| Key | Joint | Direction |
|-----|-------|-----------|
| W / S | joint1 | +/− |
| A / D | joint2 | +/− |
| Q / E | joint3 | +/− |
| I / K | joint4 | +/− |
| J / L | joint5 | +/− |
| U / O | joint6 | +/− |

**Impedance mode:** Same keys adjust the impedance target position.

**General keys:**

| Key | Function |
|-----|----------|
| Z / X | Gripper close / open |
| R | Reset all targets to zero |
| Space | Print current pose (terminal output) |

Key step size: 0.015 rad/cycle at 200 Hz (≈ 3 rad/s). Motion stops on key release.

### Waypoints & Trajectory

The Waypoints section in the connection panel supports multi-waypoint trajectory planning.

**Workflow:**

1. Adjust joint angles → click **+ Add Current** to record the current pose as a waypoint
2. Add multiple waypoints (up to 6), each with a configurable duration
3. Click **Run Trajectory** — the robot moves smoothly through all waypoints in sequence

**Implementation details:**

- Uses 7th-order polynomial interpolation (septic) for continuous position, velocity, acceleration, and jerk
- Trajectory executed in a dedicated thread at 200 Hz
- Real-time progress broadcast to the frontend

### SDK Script Runner

The toolbar **▶ Scripts** button opens a panel for running `panthera_python/scripts/` examples directly from the web UI — no command line needed.

**Available scripts (whitelist):**

| Script | Description |
|--------|-------------|
| `0_robot_get_state.py` | Read joint state (position/velocity/torque) |
| `0_robot_set_zero.py` | Set encoder zero position |
| `1_Joint_PosVel_control.py` | Per-joint position-velocity control |
| `2_inv_PosVel_control.py` | IK-based Cartesian position control |
| `3_sin_trajectory_control.py` | Sinusoidal trajectory tracking |
| `4_impedance_trajectory_control_with_gra_pd.py` | Impedance + gravity + PD trajectory control |
| `6_moveL_pos_control.py` | Cartesian straight-line position control |
| `6_moveL_rotate_control.py` | Cartesian rotation control |

**Dual execution mode:**

| Mode | Implementation | Behavior |
|------|---------------|----------|
| **Demo / simulation** | Subprocess + `PantheraSim` | Script controls virtual robot, 3D view syncs |
| **Live robot** | In-process thread + script wrapper | Uses the backend's real robot instance, drives hardware directly |

The wrapper (`_ScriptRobotWrapper`) in live mode automatically handles: stop-check on every control call, position clamping to joint limits, and state push to frontend.

### Model & File Management

- **Files panel** — Left sidebar file tree, click to switch models; supports drag-and-drop of external URDF/MJCF/STL/OBJ/DAE files
- **Structure panel** — Shows the joint-link tree as a D3.js force-directed graph
- **Visual / Collision** — Toggle between visual mesh and collision geometry display
- **Axes** — Show joint coordinate frames (red X / green Y / blue Z)
- **Shadow** — Ground shadow toggle

---

## Backend API Reference

### REST Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/config` | GET | Get robot config (joint names, limits, URDF path, current mode) |
| `/api/status` | GET | Get current state (positions, velocities, torques, script status) |
| `/api/move_joint` | POST | Move a single joint `{joint, position}` |
| `/api/move` | POST | Move all joints `{positions[], velocity?}` |
| `/api/home` | POST | Return to home (zero) position |
| `/api/stop` | POST | Hold current position |
| `/api/set_zero` | POST | Set encoder zero reference |
| `/api/set_mode` | POST | Switch control mode `{mode: "position"\|"gravity_comp"\|"impedance"}` |
| `/api/set_velocity` | POST | Set movement velocity |
| `/api/set_impedance_params` | POST | Set impedance parameters `{K[], B[], target[]?}` |
| `/api/set_impedance_target` | POST | Set impedance target position |
| `/api/waypoints` | GET | Get all waypoints |
| `/api/waypoints/add` | POST | Add a waypoint `{positions[], duration}` |
| `/api/waypoints/update` | POST | Update a waypoint |
| `/api/waypoints/delete` | POST | Delete a waypoint |
| `/api/waypoints/clear` | POST | Clear all waypoints |
| `/api/waypoints/go_to` | POST | Move to a specific waypoint |
| `/api/trajectory/run` | POST | Execute waypoint trajectory |
| `/api/trajectory/stop` | POST | Stop running trajectory |
| `/api/trajectory/status` | GET | Get trajectory progress |
| `/api/scripts` | GET | List available SDK scripts (whitelist) |
| `/api/scripts/run` | POST | Run a script `{script}` |
| `/api/scripts/stop` | POST | Stop the running script |
| `/api/script_state` | POST | Receive joint state from external simulation |
| `/api/arm_description_files` | GET | Get available URDF file list |

### WebSocket Events

**Server → Client:**

| Event | Rate | Payload |
|-------|------|---------|
| `robot_state` | 30 Hz | `{positions, velocities, torques, target_positions, control_mode, forward_kinematics, ee_position, ee_euler, external_wrench, timestamp}` |
| `config` | On connect | `{robot_name, joints, demo_mode, control_mode, impedance, end_effector_offset}` |
| `mode_changed` | On switch | `{mode}` |
| `waypoints_updated` | On change | `{waypoints}` |
| `trajectory_progress` | During execution | `{progress: 0.0~1.0}` |
| `trajectory_complete` | On finish | `{success, cancelled?}` |
| `joint_positions` | After set_zero | `{positions, velocities, torques}` |

**Client → Server:** `move_joint`, `move_all`, `home`, `stop`, `set_zero`, `set_mode`, `set_impedance_target`, `set_impedance_params`, `key_down`, `key_up`, `command`, `add_waypoint`, `run_trajectory`, `stop_trajectory`

### Control Loop Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `CONTROL_FREQ` | 200 Hz | Control command send rate |
| `BROADCAST_FREQ` | 30 Hz | WebSocket state broadcast rate |
| `END_EFFECTOR_OFFSET` | 0.07 m | Offset from Link_6 origin to tool tip |

---

## Robot Configuration

Configuration files are in `robot_param/`, in YAML format:

```yaml
robot:
  name: "Panthera-HT"
  param_file: "../robot_param/motor_param/6dof_Panthera_params_follower.yaml"
  joint_limits:
    lower: [-2.4, 0.0, 0.0, -1.6, -1.7, -2.5]
    upper: [2.4, 3.2, 4.0, 1.6, 1.7, 2.5]
  max_torque: [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]

urdf:
  file_path: "../Panthera-HT_description/urdf/Panthera-HT_description_follower.urdf"

kinematics:
  joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
```

### Config Fields

| Field | Description |
|-------|-------------|
| `robot.name` | Robot name (shown in frontend) |
| `robot.param_file` | Motor parameter file path (CAN ID, PID gains, etc.) |
| `robot.joint_limits` | Joint soft limits in radians (`lower` / `upper`, 6 values each) |
| `robot.max_torque` | Maximum torque per joint (Nm) |
| `urdf.file_path` | URDF model path (relative to the config file's directory) |
| `kinematics.joint_names` | Names of the six joints (must match URDF) |

### Built-in Configs

| File | Purpose |
|------|---------|
| `Follower.yaml` | Panthera-HT follower arm (default) |
| `xlb.yaml` | XLB robot arm |
| `motor_param/6dof_Panthera_params_follower.yaml` | Follower motor CAN parameters |
| `motor_param/6dof_Xlb_params.yaml` | XLB motor CAN parameters |

---

## Tech Stack

| Layer | Technology |
|-------|------------|
| **Backend framework** | Python 3, Flask, Flask-SocketIO, Flask-CORS |
| **Real-time communication** | WebSocket (python-socketio / socket.io-client), REST |
| **Kinematics & dynamics** | Pinocchio (pin) |
| **Numerical computation** | NumPy, SciPy |
| **Frontend framework** | Vite (ES modules, HMR) |
| **3D rendering** | Three.js, urdf-loader |
| **Graphics** | D3.js (model structure tree) |
| **Robot SDK** | hightorque_robot (pre-built wheel) |
| **Configuration** | YAML |

---

<br>

---

<br>

# Panthera 数字孪生

Panthera-HT 六轴机械臂的数字孪生系统。后端通过 Python SDK 连接真机或运行仿真，前端基于 Three.js 提供实时 3D 可视化与网页端控制。

---

## 目录

- [系统架构](#系统架构-1)
- [项目结构](#项目结构-1)
- [环境配置](#环境配置-1)
- [快速开始](#快速开始-1)
- [功能详解](#功能详解-1)
  - [连接与状态面板](#连接与状态面板-1)
  - [控制模式](#控制模式-1)
  - [关节控制](#关节控制-1)
  - [末端位姿面板](#末端位姿面板-1)
  - [力/力矩可视化](#力力矩可视化-1)
  - [键盘控制](#键盘控制-1)
  - [路点与轨迹规划](#路点与轨迹规划-1)
  - [SDK 例程运行器](#sdk-例程运行器-1)
  - [模型与文件管理](#模型与文件管理-1)
- [后端 API 参考](#后端-api-参考-1)
- [机器人配置](#机器人配置-1)
- [技术栈](#技术栈-1)

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│  浏览器 (Frontend)                                          │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐  │
│  │ 3D 渲染   │ │ 关节滑块  │ │ 力/力矩  │ │ SDK 脚本运行  │  │
│  │ Three.js │ │ 控制面板  │ │ 可视化   │ │ 器面板        │  │
│  └──────────┘ └──────────┘ └──────────┘ └───────────────┘  │
│                       │ WebSocket + REST                    │
└───────────────────────┼─────────────────────────────────────┘
                        │
┌───────────────────────┼─────────────────────────────────────┐
│  后端 (Flask)                                               │
│  ┌────────────────────┼──────────────────────────────────┐  │
│  │  app.py            │                                  │  │
│  │  ├─ REST API       │  /api/move, /api/status ...      │  │
│  │  ├─ WebSocket      │  robot_state (30Hz广播)          │  │
│  │  ├─ 控制循环        │  200Hz 主循环 (位置/重力/阻抗)   │  │
│  │  ├─ 外力估计        │  τ_ext → J^T pinv → F_ext       │  │
│  │  └─ 脚本执行器      │  白名单内 SDK 例程运行           │  │
│  └────────────────────┼──────────────────────────────────┘  │
│                       │                                      │
│  ┌────────────────────┼──────────────────────────────────┐  │
│  │  真机模式           │  仿真模式 (--demo)               │  │
│  │  Panthera SDK ←→   │  PantheraSim (Pinocchio运动学)   │  │
│  │  hightorque_robot  │  通过 HTTP 推送关节状态到后端    │  │
│  │  硬件 CAN 总线     │  3D 画面实时同步运动             │  │
│  └────────────────────┴──────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

**数据流：** 真机/仿真 → 后端 `state_broadcast_loop` (30Hz) → WebSocket `robot_state` → 前端更新 3D 模型与 UI。

---

## 项目结构

```
Panthera_digital_twin-main/
│
├── backend/                          # Python 后端
│   ├── app.py                        # 主入口 — REST API + WebSocket + 控制循环
│   ├── app_cartesian.py              # 笛卡尔阻抗控制（独立脚本）
│   ├── panthera_sim.py               # 虚拟 Panthera 类（仿真模式）
│   ├── run_script.py                 # 脚本启动器
│   └── requirements.txt              # Python 依赖
│
├── frontend/                         # Web 前端
│   ├── index.html                    # 主页面 + 全部 CSS
│   ├── package.json                  # Node 依赖
│   └── src/
│       ├── main.js                   # 应用入口 DigitalTwinApp
│       ├── robot/
│       │   └── RobotConnection.js    # WebSocket 客户端（单例）
│       ├── ui/                       # UI 组件
│       │   ├── ConnectionUI.js       # 连接面板 + 控制按钮
│       │   ├── JointControlsUI.js    # 关节滑块面板
│       │   ├── KeyboardControlUI.js  # 键盘控制
│       │   ├── ScriptControlUI.js    # SDK 例程运行面板
│       │   ├── PanelManager.js       # 浮动面板布局
│       │   └── UIController.js       # 工具栏控制
│       ├── renderer/                 # Three.js 场景渲染
│       ├── adapters/                 # 模型解析适配器
│       ├── controllers/              # 文件处理/代码编辑器
│       ├── loaders/                  # 模型加载
│       ├── models/                   # UnifiedRobotModel
│       ├── views/                    # 文件树视图
│       └── utils/                    # 工具函数 + i18n
│
├── Panthera-HT_description/          # Panthera-HT URDF + STL
├── arm_description/                  # 备用 URDF 模型
├── arm_description_xlb/              # XLB URDF 模型
└── robot_param/                      # YAML 配置文件
```

---

## 环境配置

### 系统要求

| 组件 | 要求 |
|------|------|
| 操作系统 | Linux (x86_64 / aarch64) |
| Python | 3.9+（推荐 conda 环境） |
| Node.js | 18+ |
| 包管理器 | pip + npm |

### 1. 安装后端依赖

```bash
cd backend
pip install -r requirements.txt
```

依赖包：`flask`, `flask-socketio`, `flask-cors`, `python-socketio`, `pyyaml`, `numpy`, `eventlet`。

如需连接真机，还需要安装 `hightorque_robot` SDK 和 `pin`，详见 `panthera_python/README.md`。

### 2. 安装前端依赖

```bash
cd frontend
npm install
```

核心包：`three` (3D 渲染), `urdf-loader` (URDF 解析), `socket.io-client` (实时通信), `d3` (图形), `vite` (构建)。

---

## 快速开始

### 1. 启动后端

**Demo 仿真模式（无需真机）：**

```bash
cd backend
python app.py --demo
```

**连接真机模式：**

```bash
conda activate panthera
cd backend
python app.py --config ../robot_param/Follower.yaml
```

后端默认端口 `5000`，可通过 `--port` 修改。

### 2. 启动前端

```bash
cd frontend
npm run dev
```

前端默认端口 `3000`，支持热更新。

### 3. 打开浏览器

访问 `http://localhost:3000`，页面自动加载默认 URDF 模型。点击右上角 **Connect** 连接后端即可开始使用。

---

## 功能详解

### 连接与状态面板

右上角浮动面板。连接成功后显示机器人名称、关节数、运行模式 (Live/Demo)。连接指示灯绿色=已连接。

### 控制模式

三种模式通过下拉菜单切换：

| 模式 | 原理 | 场景 |
|------|------|------|
| **Position** | 直接位置控制 `Joint_Pos_Vel(pos, vel, max_torque)` | 精确定位 |
| **Gravity** | 重力补偿 `τ = G(q)`，零力浮动 | 手动示教 |
| **Impedance** | 阻抗控制 `τ = K(q_des−q) + B(−dq) + G(q)` | 柔顺交互 |

快捷按钮：Home（回零）/ Stop（停止）/ Set Zero（编码器归零）。

### 关节控制

左侧面板，六个关节滑块。拖拽即发指令。支持弧度/角度切换、Reset 归零。连接真机时滑条边框变绿，数值 30Hz 实时刷新。

### 末端位姿面板

显示末端 XYZ 坐标和 Roll/Pitch/Yaw 欧拉角。Impedance 模式下额外显示外力/力矩估计值（带颜色编码）。

### 力/力矩可视化

仅 Impedance 模式有效。通过电机力矩估算外力：

```
τ_ext = τ_measured − G(q) − friction(dq)
F_ext = J^T (J J^T + λ²I)^(−1) τ_ext
```

3D 箭头显示：橙红色=力，青蓝色=力矩。箭头方向=方向，长度=幅值。

### 键盘控制

连接真机后键盘直接控制。W/S=joint1, A/D=joint2, Q/E=joint3, I/K=joint4, J/L=joint5, U/O=joint6, Z/X=夹爪, R=归零, Space=打印位姿。步长 0.015 rad/周期。松开即停。

### 路点与轨迹规划

最多 6 个路点，每个可设持续时间。执行时使用七阶多项式插值，200Hz 控制，实时进度广播。

### SDK 例程运行器

工具栏 **▶ Scripts** 按钮。白名单内 8 个例程可在网页上一键运行：

| 脚本 | 功能 |
|------|------|
| `0_robot_get_state.py` | 读取关节状态 |
| `0_robot_set_zero.py` | 编码器归零 |
| `1_Joint_PosVel_control.py` | 位置-速度控制 |
| `2_inv_PosVel_control.py` | 基于 IK 的笛卡尔位置控制 |
| `3_sin_trajectory_control.py` | 正弦轨迹跟踪 |
| `4_impedance_trajectory_control_with_gra_pd.py` | 阻抗轨迹控制 |
| `6_moveL_pos_control.py` | 笛卡尔直线运动 |
| `6_moveL_rotate_control.py` | 笛卡尔旋转运动 |

- **Demo 模式**：子进程 + PantheraSim 仿真运行
- **真机模式**：in-process 线程 + 封装器直接驱动硬件

### 模型与文件管理

- Files 面板：切换/拖放 URDF/STL/OBJ 模型
- Structure 面板：Joint-Link 树状结构图
- Visual/Collision：外观/碰撞体切换
- Axes：关节坐标系显示

---

## 后端 API 参考

### REST 接口

| 接口 | 方法 | 说明 |
|------|------|------|
| `/api/config` | GET | 机器人配置信息 |
| `/api/status` | GET | 当前状态（含脚本运行状态） |
| `/api/move_joint` | POST | 移动单关节 `{joint, position}` |
| `/api/move` | POST | 移动所有关节 `{positions[], velocity?}` |
| `/api/home` | POST | 回零位 |
| `/api/stop` | POST | 停止 |
| `/api/set_zero` | POST | 编码器归零 |
| `/api/set_mode` | POST | 切换控制模式 |
| `/api/set_velocity` | POST | 设置速度 |
| `/api/set_impedance_params` | POST | 设置阻抗参数 K/B |
| `/api/set_impedance_target` | POST | 设置阻抗目标 |
| `/api/waypoints` | GET | 获取路点列表 |
| `/api/waypoints/add` | POST | 添加路点 |
| `/api/waypoints/delete` | POST | 删除路点 |
| `/api/waypoints/clear` | POST | 清空路点 |
| `/api/trajectory/run` | POST | 执行轨迹 |
| `/api/trajectory/stop` | POST | 停止轨迹 |
| `/api/scripts` | GET | 列出可用脚本（白名单） |
| `/api/scripts/run` | POST | 运行脚本 |
| `/api/scripts/stop` | POST | 停止脚本 |
| `/api/script_state` | POST | 仿真状态推送 |

### WebSocket 事件

**服务端→客户端（30Hz）：**

| 事件 | 载荷 |
|------|------|
| `robot_state` | 位置/速度/力矩/目标位置/控制模式/末端位姿/外力/时间戳 |
| `config` | 机器人名/关节/模式/阻抗参数/末端偏移 |
| `mode_changed` | 新模式名 |
| `waypoints_updated` | 路点列表 |
| `trajectory_progress` | 0.0~1.0 进度 |
| `trajectory_complete` | 成功/取消 |
| `joint_positions` | 归零后状态 |

**客户端→服务端：** `move_joint`, `move_all`, `home`, `stop`, `set_zero`, `set_mode`, `set_impedance_target`, `set_impedance_params`, `key_down`, `key_up`, `command`, `add_waypoint`, `run_trajectory`, `stop_trajectory`

### 控制参数

| 参数 | 值 | 说明 |
|------|-----|------|
| CONTROL_FREQ | 200 Hz | 控制指令频率 |
| BROADCAST_FREQ | 30 Hz | 状态广播频率 |
| END_EFFECTOR_OFFSET | 0.07 m | 工具尖端偏移 |

---

## 机器人配置

YAML 配置文件位于 `robot_param/`：

```yaml
robot:
  name: "Panthera-HT"
  param_file: "../robot_param/motor_param/6dof_Panthera_params_follower.yaml"
  joint_limits:
    lower: [-2.4, 0.0, 0.0, -1.6, -1.7, -2.5]
    upper: [2.4, 3.2, 4.0, 1.6, 1.7, 2.5]
  max_torque: [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]

urdf:
  file_path: "../Panthera-HT_description/urdf/Panthera-HT_description_follower.urdf"

kinematics:
  joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
```

| 字段 | 说明 |
|------|------|
| `robot.param_file` | 电机参数文件（CAN ID, PID 等） |
| `robot.joint_limits` | 关节软限位 (rad) |
| `robot.max_torque` | 各关节最大力矩 (Nm) |
| `urdf.file_path` | URDF 路径（相对配置文件所在目录） |
| `kinematics.joint_names` | 关节名（须与 URDF 一致） |

---

## 技术栈

| 层 | 技术 |
|----|------|
| **后端** | Python 3, Flask, Flask-SocketIO, Pinocchio, NumPy, SciPy |
| **前端** | Vite, Three.js, urdf-loader, Socket.IO Client, D3.js |
| **机器人** | hightorque_robot SDK, Panthera-HT 六轴机械臂 |
| **配置** | YAML |
