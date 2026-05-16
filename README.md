# DogMatch — 机器狗足球赛策略代码

CyberDog 2 足球赛前锋（Goaler）与守门员（Keeper）的 ROS 2 策略代码。

## 目录结构

```
repo/
├── Goaler/                 # 前锋 (部署到 dog1)
│   ├── ball_kicker.py      # 主状态机: Approch → Align → Dash
│   ├── approach_controller.py  # 接近球: 导航到预踢点
│   ├── position_adjuster.py    # 对齐: 调整位姿到踢球线
│   ├── dash_kicker.py          # 冲刺: 推球
│   ├── vrpn_perception.py      # VRPN 动捕感知
│   ├── setup.py                # ROS 2 包配置 (entry: striker)
│   ├── start.sh                # 赛方标准启动脚本
│   └── run_kick_on_dog.sh      # 开发/测试用启动脚本
├── Keeper/                 # 守门员 (部署到 dog2)
│   ├── run_goalkeeper.py       # 主节点: 守门状态机
│   ├── goalkeeper_controller.py # 扑救控制器
│   ├── goal_line_patrol.py     # 门线巡逻
│   ├── vrpn_perception.py      # VRPN 动捕感知
│   └── setup.py                # ROS 2 包配置 (entry: goalkeeper)
└── README.md
```

## 部署

### 环境

- ROS 2 Galactic
- CyberDog 2 固件（M813 运控板 + NX 应用板）
- VRPN 动捕系统（另终端启动）

### 工作空间路径

代码放入每只狗的 `~/ros2_ws/src/demo_python_pkg/demo_python_pkg/`。
`setup.py` 和 `package.xml` 放在 `~/ros2_ws/src/demo_python_pkg/`。

```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select demo_python_pkg
```

### VRPN 提前启动

```bash
ssh mi@<狗IP>
cd /home/mi/vrpn_client_ros2
source /opt/ros2/galactic/setup.bash
source /opt/ros2/cyberdog/local_setup.bash
source src/install/setup.bash
ros2 launch vrpn_listener sync_entity_state.launch
```

### 赛时启动

```bash
# 前锋 (dog1)
ssh mi@<狗1IP>
cd /home/mi && ./start.sh dog1 goal_right   # 或 goal_left

# 守门员 (dog2)
ssh mi@<狗2IP>
cd /home/mi && ./start.sh dog2 goal_left    # 或 goal_right
```

## 前锋状态机

```
APPROACH → ALIGN_KICK → DASH (持续)
  ↑           ↑            │
  └──超时─────┘            │
  ↑                        │
  └────球丢失──────────────┘
                           │
                    距球门 ≤0.5m → 站立停止
```

### 各阶段说明

| 阶段 | 功能 | 步态 |
|------|------|------|
| **APPROACH** | 导航到球后方预踢点，面朝球 | FAST_TROT (305) |
| **ALIGN_KICK** | 调整位姿对齐踢球线，连续3帧对齐触发 | MEDIUM_TROT (308) |
| **DASH** | 胸口推球向球门，开头0.3s微侧推避免压球 | FAST_TROT (305) + 内八步态 |

### 关键参数 (可通过 ROS param 覆盖)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `goal_tracker` | goal_right | VRPN 球门追踪器名 |
| `dash_speed` | 2.0 | 冲刺线速度 (m/s) |
| `dash_centroid_z` | 0.10 | 冲刺重心高度 (ROS桥可能不转发) |
| `dash_nudge_vy` | 0.15 | 冲刺开头侧推速度 (m/s) |
| `dash_nudge_duration` | 0.30 | 侧推持续时间 (s) |
| `dash_stop_goal_dist` | 0.50 | 距球门停止距离 (m) |
| `max_yaw_rate` | 0.6 | 最大转向角速度 (rad/s) |
| `align_consecutive_frames` | 3 | 连续对齐帧数 |
| `align_ready_relax` | 1.0 | 对齐松弛因子 |
| `align_max_lateral_speed` | 0.25 | 对齐最大横向速度 |
| `align_timeout` | 6.0 | 对齐超时 (s) |
| `approach_timeout` | 30.0 | 接近超时 (s) |
| `safe_ball_dist` | 0.35 | 安全球距 (太近则重来) |

## 守门员

状态机包含门线巡逻、球追踪、扑救等逻辑。详见 `Keeper/run_goalkeeper.py`。

## LCM 层说明

机器人的运控指令最终通过 LCM 协议（`robot_control_cmd_lcmt` 结构体）
发送到 M813 运控板。ROS `MotionServoCmd` 由系统桥接转换为 LCM。

LCM 文档参考: https://miroboticslab.github.io/blogs/#/cn/cyberdog_loco_cn

`pos_des[2]` 字段在 LCM 层可直接控制身体质心高度 (0.10~0.32m)，
但在 ROS 桥接层标注为"暂不开放"，可能不转发。
当前通过 `value` 字段 bit1 (0=内八步态低站姿, 1=垂直步态) 间接降低重心。
