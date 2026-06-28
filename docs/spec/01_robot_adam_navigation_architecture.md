# Robot Adam 宏观架构规格说明书 (SPEC 01_Macro)

> **版本**: v1.0 | **日期**: 2026-06-28 | **状态**: Released
> **存放目录**: `docs/spec/01_robot_adam_navigation_architecture.md`
> **范围**：全局软件蓝图、分层架构、数据流向边界与工作空间组织结构。本文件不包含任何具体算法的调优参数或代码级细节。
> **关联说明**：本文件是系列 SPEC 的最高指导纲领。下属微观执行层 SPEC 02~05 必须严格遵循本文确立的单向演进依赖图进行筑基开发。

---

## 1. 全局纵向分层拓扑与边界 (Top-Down Landscape)

Robot Adam 系统从顶层到底层严格划分为 5 个纵向功能层与 1 个横向资产管理中心。层与层之间仅允许通过标准的 ROS2 接口（Action / Service / Topic）进行单向或请求响应式通信，严格禁止跨层越权调用和硬件驱动直接对接高层决策。

```
====================================================================================
               Level 5: 具身智能语义决策层 (adam_vln) -> (纯数据与逻辑层)
            [人类模糊自然语言指令] -> [Ollama/大模型任务拆解] -> [原子 Action 序列]
====================================================================================
                                      │ (ROS2 Action / Task Tree)
                                      ▼
====================================================================================
         Level 4: 机器人控制中枢与状态调度层 (adam_hub_controller) -> (业务中枢)
            [ROS2 Lifecycle 托管状态机] -> 动态装载/卸载建图、探索、已知图导航模块
====================================================================================
                                      │ (Lifecycle State Transitions)
                                      ▼
====================================================================================
       Level 3: 可插拔多算法导航与立体避障层 (adam_bringup / adam_navigation)
   ┌───────────────────┬───────────────────┬─────────────────────┬──────────────────┐
   │  2D 几何导航流水线 │  3D 激光导航流水线 │  传统视觉 VSLAM 流水线│ 神经网络 SLAM 流水线│
   │ Cartographer/Nav2 │  FAST-LIO2 / STVL │  ORB-SLAM3 / RTAB    │ DROID-SLAM 稠密网 │
   └───────────────────┴───────────────────┴─────────────────────┴──────────────────┘
====================================================================================
                                      │ (标准化 /cmd_vel 与全局寻路)
                                      ▼
====================================================================================
          Level 2: 多源传感器状态估计层 (adam_localization) -> (常驻地基层)
         [双层 EKF 状态估计器] -> 强力锁死并融合各路 VO/LIO、车轮里程计与高频 IMU
====================================================================================
                                      │ (发布绝对稳定、不断流的 map -> odom -> base_link TF)
                                      ▼
====================================================================================
           Level 1: 物理硬件与常驻驱动/仿真层 (robot_description) -> (物理常驻层)
     [Gazebo 仿真 / 真车硬件接口] -> (2WD/4WD/Omni底盘) + (Mid360 / 2D雷达 / 相机)
====================================================================================

    ============================================================================
    ★ 横向支撑中心 ★：统一的数据资产与地图集中管理系统 (adam_assets)
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

## 2. 统一工作空间组织结构 (Workspace Layout)

为保证整个导航系统的模块内聚性，工作空间内的功能包组织结构必须严格按照以下边界和依赖规范划分，禁止在各包之间引入交叉循环依赖（Circular Dependencies）。

```
robot_adam/
├── build_sim.sh                        # 统一仿真一键编译脚本
└── src/
    ├── 1.simulation/                   # Level 1: 仿真物理与机器人描述层（常驻底层）
    │   └── robot_description/          # 6 种运控变体底盘、Xacro 运动学宏、Gazebo 插件配置
    │
    ├── 2.localization_mapping/         # Level 2 & Level 3 的传感器状态估计与多源建图流水线
    │   ├── adam_localization/          # [核心] 常驻多源状态估计融合包（实现双层 EKF 里程计底座）
    │   ├── adam_slam/                  # 激光建图与快速重定位算法集成（2D Cartographer / 3D FAST-LIO2）
    │   ├── adam_vision/                # 传统特征点视觉 SLAM 与目标识别（ORB-SLAM3 封装）
    │   └── adam_neural_slam/           # 神经网络与端到端深度学习 SLAM 包（DROID-SLAM 共享内存外壳）
    │
    └── 3.navigation_ai/               # Level 4 & Level 5 的核心控制、资产管理、时空规划与具身大脑
        ├── adam_assets/                # [横向中心] 全局静态地图、相机内参、AI拓扑字典等非结构化资产包
        ├── adam_bringup/               # 顶层单点与集群一键启动配置层（Launch 集合）
        ├── adam_hub_controller/        # 机器人中枢控制器（ROS2 Lifecycle 状态机业务总揽）
        ├── adam_navigation/            # 传统几何导航与时空规划核心包（Nav2 深度集成与行为树定制）
        └── adam_vln/                   # 视觉语言导航与具身智能本地 LLM 决策调度层
```

### 2.1 包依赖方向约束

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

## 3. 全栈技术选型矩阵 (Technology Full-View)

宏观框架下各层组件的技术选型及方案对齐如下，各模块的具体开发规范由微观执行 SPEC 详细定义：

| 维度 | 技术组件 / 开源库选型 | 核心工程价值与用途 |
| --- | --- | --- |
| **基础非结构化数据** | ROS2 `ament_index_python` 路由 | 资产集中式版本化管理，杜绝绝对路径硬编码 |
| **底层状态估计** | `robot_localization` (Dual EKF) | 维护绝对连续、高频、死不断流的整车基础位姿感知 |
| **2D 几何导航栈** | Cartographer + `explore_lite` + Nav2 | 跑通扫地机核心闭环：自动探图 → 归档 → 一帧重定位 → 寻路 |
| **3D 空间几何栈** | FAST-LIO2 + `pointcloud_to_laserscan` | 点云高频状态估计，高度切片降维供全局寻路安全边界 |
| **立体避障层** | Nav2 Costmap 3D + STVL 插件 | 基于 OpenVDB 结构与时空衰减机制，完美消除动态人流残影 |
| **多模态视觉定位** | ORB-SLAM3 (Stereo-Inertial 模式) | 利用 DBoW 词袋模型提供极强的回环修正与拍照即锁死定位能力 |
| **端到端神经定位** | DROID-SLAM (稠密光流神经网络) | 具备具身智能前沿状态估计水平，免疫剧烈运动模糊与低纹理大白墙 |
| **开放域视觉感知** | YOLOv11 / YOLO-World | 零样本实时提取视野中物体标签，解算并反投影 3D 世界坐标 |
| **具身智能语义层** | Ollama + Qwen-2.5-Instruct (本地化) | 接收模糊自然语言指令，联动语义拓扑字典，自动拆解输出标准任务树 |

---

## 4. 下属详细执行 SPEC 单向演进依赖图 (Bottom-Up Roadmap)

微观执行层 SPEC 明确抛弃"中枢先行、算法后挂"的错误研发顺序，严格遵循"数据与算法流水线一层一层向上筑基，控制中枢最后收官总揽"的工程学确定性路线进行单向演进开发。

```
    ┌─────────────────────────────────────────────────────────────────────┐
    │  SPEC 01: 宏观架构规格书 (PRD纲领 / Released)                       │
    │  └── 顶层蓝图、分层边界、包依赖规范                                  │
    └─────────────────────────────────────────────────────────────────────┘
                                      │ 继承蓝图
                                      ▼
    ┌─────────────────────────────────────────────────────────────────────┐
    │  SPEC 02: 基础设施与 2D 几何导航栈 (Working)                        │
    │  资产骨架 → 双层 EKF → Cartographer 建图/定位 → explore_lite →     │
    │  地图归档 → 一帧重定位 → Nav2 闭环                                   │
    └─────────────────────────────────────────────────────────────────────┘
                                      │ 底层 2D 打通后升维 3D 空间
                                      ▼
    ┌─────────────────────────────────────────────────────────────────────┐
    │  SPEC 03: 3D 空间几何与时空立体避障 (Planned)                       │
    │  FAST-LIO2 高频里程计 → pointcloud_to_laserscan 切片 →              │
    │  STVL 时空体素 1s 消散 → 3D Nav2 闭环                               │
    └─────────────────────────────────────────────────────────────────────┘
                                      │ 引入多模态冗余视觉源
                                      ▼
    ┌─────────────────────────────────────────────────────────────────────┐
    │  SPEC 04: 多模态视觉感知与断流融合栈 (Planned)                       │
    │  相机标定资产 → ORB-SLAM3 VIO → DROID-SLAM 推理壳 →                │
    │  grid_projector → 多源 EKF 终极合流 → 关灯不撞墙                     │
    └─────────────────────────────────────────────────────────────────────┘
                                      │ 算法全线筑基完毕，中枢收官调度
                                      ▼
    ┌─────────────────────────────────────────────────────────────────────┐
    │  SPEC 05: 控制中枢与大模型具身决策 (Planned)                         │
    │  Lifecycle 状态机 → YOLO + 3D 投影 → 语义拓扑字典 →                  │
    │  Ollama 任务拆解 → 全栈终极铁律："去桌子那里" → 自主导航             │
    └─────────────────────────────────────────────────────────────────────┘
```

**开发红线**：禁止在低层算法（如 2D/3D 里程计、避障层）未通过其特定通关标准（Gate Criteria）之前，编写或注入高层中枢（Level 4 / Level 5）的状态控制逻辑。

---

## 5. 全局系统设计准则 (System Design Principles)

为确保整个工程交付质量，后续微观 SPEC 的 Plan 拆解与具体编码阶段必须无条件死守以下准则：

### 5.1 TF 树广播权唯一性

任何时候，有且仅有一个节点拥有 `map -> odom` 的广播权。所有 SLAM、VSLAM、Neural SLAM 功能包在配置中必须将其 `publish_tf` 或 `provide_odom_frame` 强制设为 `false`，仅输出 Pose 话题。TF 的广播权统一且唯一收拢到 `adam_localization` 层的全局 EKF 节点。

### 5.2 常驻死忠层与动态调度层物理分离

Level 1（驱动与仿真）与 Level 2（状态估计）属于常驻死忠层，开机必须常驻且断不断流。Level 3、Level 4 与 Level 5 属于业务功能与调度层，各节点的生命周期完全受控于中枢，允许随业务场景动态启动、挂起或杀死，其任何状态的切换绝不能引起底层里程计数据流和 TF 树的真空期。

### 5.3 多源定位健康度监控与降级保护

高层 Global EKF 必须对上层可插拔的各路"明星算法位姿"进行实时的协方差矩阵监控。一旦发生物理极限场景下的退化（如激光在长走廊退化、视觉在全黑暗室 Lost），必须在毫秒级自动剔除异常源，闭环权重无缝切向常驻的底层轮速+IMU 里程计进行保守盲开，实行故障安全行为（Failure-Safe）。

---

## 6. 系列 SPEC 索引

| 文件 | 范围 | 状态 | 前置依赖 | 原子单元数 |
|------|------|------|---------|-----------|
| `01_robot_adam_navigation_architecture.md` | 宏观蓝图、分层、包结构 | **Released** | 无 | — |
| `02_infra_and_2d_geometry.md` | adam_assets 骨架 + 2D EKF + Cartographer + explore_lite + Nav2 | **Working** | 01 | 6 |
| `03_3d_spatial_and_stvl.md` | FAST-LIO2 + pointcloud_to_laserscan + STVL 时空避障 | **Planned** | 02 | 3 |
| `04_multimodal_vision_slam.md` | ORB-SLAM3 + DROID-SLAM + grid_projector + 终极 EKF 融合 | **Planned** | 02 | 4 |
| `05_central_hub_and_vln.md` | Lifecycle 状态机 + YOLO + 语义拓扑 + Ollama VLN | **Planned** | 02+03+04 | 4 |

---

## 7. 依赖仓库全景

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
