# Robot Adam 宏观架构规格说明书 (SPEC 01_Macro)

> 版本: v1.0 | 日期: 2026-06-28 | 状态: Released
> 范围：全局软件蓝图、分层架构与包组织结构边界。不包含任何算法级实现细节。
> 关联：本文件是系列 SPEC 的顶层索引，下属 SPEC 02~05 按 Bottom-Up 顺序依次展开。

---

## 1. 系统全局分层拓扑

系统严格遵循纵向 5 层与横向 1 中心设计。层与层之间通过标准 ROS2 接口（Topic/Action/Service）解耦，禁止越层调用。

```
====================================================================================
               Level 5: 具身智能语义决策层 (adam_vln)
            [人类模糊指令] -> [大模型任务拆解] -> [原子 Action 序列]
====================================================================================
                                      │ (ROS2 Action / Service)
                                      ▼
====================================================================================
              Level 4: 机器人控制中枢与状态调度层 (adam_hub_controller)
            [ROS2 Lifecycle 托管状态机] -> 动态装载/卸载建图、探索、导航模块
====================================================================================
                                      │ (Lifecycle State Transitions)
                                      ▼
====================================================================================
        Level 3: 可插拔多算法导航与立体避障层 (adam_bringup / adam_navigation)
   [2D 激光栈]  │  [3D 激光栈]  │  [传统 VSLAM]  │  [Neural SLAM]
====================================================================================
                                      │ (标准化 /cmd_vel 与全局位姿)
                                      ▼
====================================================================================
              Level 2: 多源传感器状态估计层 (adam_localization)
            [双层 EKF] -> 融合 VO/LIO/轮速计/IMU，发布唯一 map->odom TF
====================================================================================
                                      │ (发布唯一 map -> odom -> base_link TF)
                                      ▼
====================================================================================
         Level 1: 物理硬件与常驻驱动/仿真层 (robot_description)
          [Gazebo Classic 11 / 真车硬件接口] -> (2WD/4WD/Omni 底盘 + 传感器)
====================================================================================

    ============================================================================
    ★ 横向支撑中心 ★：统一数据资产与地图集中管理系统 (adam_assets)
    集中式、版本化管理全栈算法产出的所有非结构化资产（.yaml / .pgm / .pcd / .db / .json）
    ============================================================================
```

### 1.1 层间通信契约

| 层级 | 向下输出 | 向上输出 | 接口类型 |
|------|---------|---------|---------|
| L1 物理层 | `/odom`, `/imu/data`, `/scan`, `/tf_static` | — | Topic |
| L2 状态估计 | `map->odom->base_link` TF | 融合后的 `odom` | TF + Topic |
| L3 导航层 | `/cmd_vel` | `/map`, `/goal_reached` | Topic + Action |
| L4 中枢层 | Lifecycle 状态切换 | 模式状态、传感器健康度 | Service + Topic |
| L5 语义层 | 目标点 Action 请求 | 任务状态反馈 | Action |

---

## 2. 架构设计原则

### 2.1 设计目标

基于 ROS2 Humble 与 Nav2 异步流水线，构建"多源算法对齐、资产集中管理、开机秒级重定位、三维立体避障、AI 具身决策分层"的高层导航框架：

- **多体系算法基座**：横向支持 2D/3D 激光、传统 VSLAM（ORB-SLAM3/VINS）、神经网络 SLAM（DROID-SLAM）。
- **统一资产仓库**：`adam_assets` 包集中管理所有算法的配置、地图、标定、语义拓扑等非结构化资产。
- **Sim-to-Real 透明切换**：传感器驱动与仿真接口对齐，上层算法 100% 不感知底层驱动模式。
- **生命周期中枢**：ROS2 Lifecycle Node 机制，支持 APP/Web/大模型下发状态切换指令。

### 2.2 全局硬约束

以下约束在任何算法栈组合下均必须遵守：

- **TF 树广播权唯一性**：任何时候，有且仅有一个节点拥有 `/tf` 中 `map -> odom` 的广播权。所有 SLAM/VSLAM 节点必须将 `publish_tf` 设为 `false`，将其位姿作为 `nav_msgs/msg/Odometry` 话题输出给 `adam_localization`，由 EKF 统一发布唯一的 `map -> odom` TF。
- **传感器退化降级**：任何一路传感器 Lost 时（通过协方差阈值监控），系统必须自动降级至更保守的融合模式，不得瞬间失控。

---

## 3. 算法栈全景总览

| 栈 | 建图 | 定位 | 探索 | 全局规划 | 局部控制 | 避障 |
|----|------|------|------|---------|---------|------|
| 2D 激光 | Cartographer | Cartographer 纯定位 | explore_lite | Smac 2D | MPPI | 2D Costmap |
| 3D 激光 | FAST-LIO2 | FAST-LIO2 | pointcloud_to_laserscan + explore_lite | Smac 3D | MPPI | STVL |
| 传统 VSLAM | ORB-SLAM3 | ORB-SLAM3 + 词袋重定位 | — | Smac 2D | MPPI | 2D/3D Costmap |
| Neural SLAM | DROID-SLAM | DROID-SLAM 稠密光流 | — | Smac 2D | MPPI | grid_projector |

> 详细参数配置与 Launch 文件定义见各子 SPEC（02~05）。

---

## 4. 统一工作空间组织结构

```
robot_adam/
├── build_sim.sh                         # 一键编译脚本
└── src/
    ├── 1.simulation/                    # 仿真物理与描述层（常驻底层）
    │   └── robot_description/          # [已收官] 6种运控变体 + Xacro + Gazebo 插件 + World
    │
    ├── 2.localization_mapping/          # 状态估计、多源建图与多模态感知层
    │   ├── adam_slam/                  # 激光建图与快速重定位（2D Cartographer / 3D FAST-LIO2）
    │   ├── adam_vision/                # 传统视觉 SLAM（ORB-SLAM3）与 YOLO 目标识别
    │   ├── adam_neural_slam/           # 神经网络 SLAM（DROID-SLAM 推理外壳 + grid_projector）
    │   └── adam_localization/          # 核心多源传感器融合包（双层 EKF）
    │
    └── 3.navigation_ai/                 # 核心控制、资产、规划与大模型决策层
        ├── adam_assets/                # 全局静态地图与算法资产配置中心仓库
        ├── adam_bringup/               # 全局启动与中枢调度层
        ├── adam_hub_controller/        # 机器人中枢生命周期控制器
        ├── adam_navigation/            # Nav2 二次封装（Smac + MPPI + STVL）
        └── adam_vln/                   # 视觉语言导航与具身智能决策（Ollama + 语义拓扑）
```

### 4.1 包依赖方向约束

```
robot_description (L1)  →  adam_localization (L2)  →  adam_slam / adam_vision / adam_neural_slam (L3)
                                                          ↓
adam_navigation (L3)  ←  adam_bringup (L3)  ←  adam_hub_controller (L4)
                                                          ↓
                                                  adam_vln (L5)
                                                          ↓
                                                  adam_assets (横向，被所有上层包引用)
```

禁止跨层反向依赖（如 `adam_vln` 直接引用 `robot_description`）。

---

## 5. 系列 SPEC 索引

| 文件 | 范围 | 状态 | 前置依赖 |
|------|------|------|---------|
| `01_macro_architecture.md` | 宏观蓝图、分层、包结构 | **Released** | 无 |
| `02_infra_and_2d_geometry.md` | adam_assets 骨架 + 2D EKF + Cartographer 建图/定位 + explore_lite + Nav2 | Working | 01 |
| `03_3d_spatial_and_stvl.md` | FAST-LIO2 + pointcloud_to_laserscan + STVL 时空避障 | Planned | 02 |
| `04_multimodal_vision_slam.md` | ORB-SLAM3 + DROID-SLAM + grid_projector | Planned | 02 |
| `05_central_hub_and_vln.md` | adam_hub_controller Lifecycle 状态机 + YOLO + Ollama VLN | Planned | 02+03+04 |

---

## 6. 依赖仓库全景

| 分类 | 仓库 | 用途 | 维护状态 |
|------|------|------|---------|
| 2D SLAM | [Cartographer ROS2](https://github.com/ros2/cartographer_ros) | 2D 建图与分支定界重定位 | 活跃 |
| 探索 | [m-explore (explore_lite)](https://github.com/hrnr/m-explore/tree/ros2) | 边界前沿无图探索 | 社区维护 |
| 导航 | [Navigation2](https://github.com/ros-navigation/navigation2) | Smac 规划器 + MPPI + BT | 活跃 |
| 3D LIO | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) | Mid360 雷达惯导里程计 | ⚠ 官方停更，需 ROS2 社区 fork |
| 避障 | [STVL](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) | 3D 时空体素衰减 | 维护中 |
| 降维 | [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan) | 3D 点云 → 2D 伪激光 | 维护中 |
| VSLAM | [ORB-SLAM3 ROS2](https://github.com/thien94/ORB_SLAM3_ROS2) | 视觉惯导 VSLAM | 社区维护 |
| VIO 备选 | [VINS-Mono ROS2](https://github.com/TechColonial/VINS-Mono-ROS2) | 滑动窗口 VIO | 社区维护 |
| Neural SLAM | [DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM) | 稠密光流 SLAM | ⚠ 需独立 C++ 推理进程（TensorRT），禁 ROS2 回调中直接推理 |
| 地图桥接 | [RTAB-Map ROS2](https://github.com/introlab/rtabmap_ros) | 稠密回环检测与地图桥接 | 活跃 |
| 目标检测 | [Ultralytics YOLO](https://github.com/ultralytics/ultralytics) | YOLOv11/World 开放域检测 | 活跃 |
| 大模型 | [Ollama](https://github.com/ollama/ollama) | 本地 LLM 推理（Qwen-2.5） | 活跃 |
