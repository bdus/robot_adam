# 机器人中枢控制器与大模型语义决策详细规格说明书 (SPEC 05_Hub_VLN)

> 版本: v1.0 | 日期: 2026-06-28 | 状态: Working
> 范围：adam_hub_controller Lifecycle 状态机 + YOLO 开放域目标检测 + 语义拓扑字典 + Ollama VLN 任务拆解
> 前置依赖：SPEC 02（2D 全栈）、SPEC 03（3D 避障）、SPEC 04（视觉 SLAM）— 底层算法流水线已全部单点调通
> 总工期预估：~3 天 | 原子单元数：3

---

## 1. 范围与边界

### 1.1 本期目标

收官总揽：编写 `adam_hub_controller` LifecycleNode 状态机，挂载并动态受控激活底层所有算法节点；集成 YOLO 开放域物体检测解算 3D 世界坐标并沉淀语义拓扑字典；通过 Ollama 本地大模型解析模糊自然语言，驱动 Nav2 Action 完成具身任务全链条打通。

### 1.2 本期不包含

- 新的底层算法流水线开发（2D/3D/视觉均已在前序 SPEC 完成）
- 地图归档机制（SPEC 02 Unit 4 已完成）

### 1.3 输入依赖

| 依赖 | 来源 | 状态 |
|------|------|------|
| Cartographer 建图/定位 | SPEC 02 | 需先完成 |
| `local_ekf_node` + `global_ekf_node` | SPEC 02 | 需先完成 |
| FAST-LIO2 3D 里程计 | SPEC 03 | 需先完成 |
| STVL 时空避障 | SPEC 03 | 需先完成 |
| ORB-SLAM3 / DROID-SLAM | SPEC 04 | 需先完成 |
| 相机图像话题 | robot_description | ✅ 已完成 |
| `adam_assets/maps_semantic/` | 本 SPEC 创建 | 本期待产出 |

---

## 2. 原子单元拆解

本期拆解为 3 个最小可交付单元。

### 📦 Unit 1: adam_hub_controller Lifecycle 状态机

**功能描述**：控制中枢。它不参与任何数学计算，但死死管住底层 Cartographer、FAST-LIO2、Nav2 等所有计算节点的生命周期。基于 ROS2 `LifecycleNode` 机制实现。

**输入依赖**：无（它管理所有节点，但不依赖它们才能启动）。

**产出物**：
- `adam_hub_controller/adam_hub_controller/__init__.py`
- `adam_hub_controller/adam_hub_controller/hub_manager_node.py`：Lifecycle 节点主实现
- `adam_hub_controller/adam_hub_controller/state_machine.py`：状态机逻辑
- `adam_hub_controller/srv/`：自定义服务定义
- `adam_hub_controller/package.xml`
- `adam_hub_controller/setup.py`

**状态机核心逻辑**：
```
[Unconfigured] (系统冷启动)
    │
    ▼ on_configure()
[Inactive] (中枢就绪，静默等待指令)
    │
    ├──► on_activate() [探索建图模式]
    │      激活: cartographer_mapping, explore_lite, nav2
    │      挂起: cartographer_localization
    │
    ├──► on_activate() [已知图导航模式]
    │      激活: cartographer_localization, nav2
    │      挂起: cartographer_mapping, explore_lite
    │
    └──► on_activate() [3D 避障模式]
           激活: fast_lio2, stvl_costmap, nav2_3d
           挂起: cartographer_mapping
```

**服务接口**：
| 接口 | 类型 | 说明 |
|------|------|------|
| `/adam_hub/switch_mode` | `std_srvs/srv/SetBool` | `True`=探索建图, `False`=已知图导航 |
| `/adam_hub/save_current_map` | `std_srvs/srv/Trigger` | 触发地图归档至 `adam_assets` |
| `/adam_hub/sensor_status` | Topic | 发布各传感器健康度状态 |

**通关标准**：
1. 通过 `ros2 service call /adam_hub/switch_mode` 切换模式
2. 底层子节点严格按照矩阵进行 `Active` ↔ `Inactive` 无缝跳转
3. 无 Ghost 进程残留（`ros2 node list` 验证）

---

### 📦 Unit 2: YOLO 物体 3D 射线投影与语义拓扑字典

**功能描述**：小车消费单目/RGBD 图像，YOLOv11/YOLO-World 实时框出物体（如"冰箱"、"桌子"）。利用当前的常驻全局平滑 TF（来自 `global_ekf_node`），将 2D 像素框的中心点通过深度数据反投影解算出在 `map` 坐标系下的绝对 [x, y, z] 坐标。

**输入依赖**：
- `/camera/image_raw`：图像话题
- `/camera/depth`：深度话题（或 Mid360 雷达点云）
- `map -> base_link` TF（来自 `global_ekf_node`）

**产出物**：
- `config/yolo_params.yaml`：YOLO 模型配置（模型权重路径、置信度阈值、类别过滤）
- `launch/yolo_detector.launch.py`：启动脚本
- `adam_vln/adam_vln/semantic_map.py`：语义拓扑字典维护节点

**3D 投影算法**：
```
1. YOLO 检测到物体 2D 边界框
2. 取边界框中心点像素坐标 (u, v)
3. 查询深度图/点云，获取该像素的深度值 d
4. 利用相机内参矩阵 K，将 (u, v, d) 转换为相机坐标系 3D 点 P_cam
5. 利用 TF tree (camera_link → base_link → map)，将 P_cam 转换到 map 坐标系
6. 得到物体在 map 下的绝对坐标 (x, y, z)
```

**语义拓扑字典 JSON 格式**：
```json
{
  "world": "bigH",
  "timestamp": "2026-06-28T16:00:00Z",
  "objects": [
    {"label": "refrigerator", "position": [1.5, 2.3, 0.0], "confidence": 0.92},
    {"label": "table", "position": [3.1, -1.2, 0.0], "confidence": 0.88},
    {"label": "chair", "position": [2.8, -1.5, 0.0], "confidence": 0.85}
  ]
}
```

**持久化路径**：`adam_assets/maps_semantic/bigH_world_topology.json`

**通关标准**：
1. YOLO 正确框出仿真场景中的物体
2. 解算出的物体坐标在 Rviz 中以 Marker 正确显示
3. JSON 文件正确写入 `adam_assets/maps_semantic/`

---

### 📦 Unit 3: Ollama 具身智能 Prompt 适配器与 Nav2 Action 驱动

**功能描述**：本地部署 `Qwen-2.5-Instruct`，接收人类模糊自然语言指令（如"去冰箱那里"），通过结构化 Prompt 强制大模型输出标准 JSON 任务树，最终调用 Nav2 Action 驱动机器人。

**输入依赖**：
- `adam_assets/maps_semantic/bigH_world_topology.json`（来自 Unit 2）
- Nav2 `NavigateToPose` Action 服务

**产出物**：
- `adam_vln/adam_vln/llm_reasoner.py`：Ollama 通信适配器与 Prompt 解析器
- `adam_vln/adam_vln/vln_server_node.py`：语义导航主节点

**任务链闭环**：
```
[人类输入] "我有点渴，去冰箱那"
    │
    ▼
llm_reasoner.py 构造 Prompt（含 topology.json 上下文）
    │
    ▼
Ollama (Qwen-2.5-Instruct) 推理
    │
    ▼
输出 JSON 任务树：
{
  "intent": "navigate_to_object",
  "target": "refrigerator",
  "target_coordinates": [1.5, 2.3, 0.0],
  "fallback": "如果冰箱不在视野中，搜索最近相似物体"
}
    │
    ▼
vln_server_node.py 解析 JSON
    │
    ▼
调用 Nav2 NavigateToPose Action → 底盘运动
    │
    ▼
到达目标 → 发布 "task_complete" 状态
```

**结构化 Prompt 设计**：
```
你是一个机器人任务规划器。以下是可以到达的物体列表：
{topology_json}

用户说："{user_input}"

请输出 JSON 任务树，格式为：
{
  "intent": "navigate_to_object | navigate_to_pose | unknown",
  "target": "物体名称",
  "target_coordinates": [x, y, z],
  "fallback": "备选方案描述"
}

只输出 JSON，不要附加任何解释。
```

**通关标准**：
1. 终端输入"去桌子那里"，Ollama 准确吐出桌子的坐标 JSON
2. `vln_server_node.py` 正确解析 JSON 并调用 Nav2 Action
3. 小车平滑启动，自动绕过路上临时放置的障碍物，最终精准停在桌子前方 50cm 处
4. 系统数据、资产、运控全线大闭环

---

## 3. 严格 Bottom-Up 构建与调试顺序

```
步骤 1: Unit 1 中枢状态机收官 (1天)
    ↓
步骤 2: Unit 2 YOLO 检测 + 语义拓扑 (1天)
    ↓
步骤 3: Unit 3 Ollama 大模型具身推理 (1天)
```

### 🛠️ 步骤 1：中枢状态机 — 1 天

**操作**：
1. 编写 `hub_manager_node.py`，基于 `rclpy_lifecycle.LifecycleNode`
2. 实现 `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`
3. 定义模式切换服务
4. 通过命令行模拟切换测试

**通关验证**：
```bash
ros2 service call /adam_hub/switch_mode std_srvs/srv/SetBool "{data: true}"
ros2 node list   # 确认对应节点被激活
ros2 service call /adam_hub/switch_mode std_srvs/srv/SetBool "{data: false}"
ros2 node list   # 确认节点切换无 Ghost 残留
```

### 🛠️ 步骤 2：YOLO + 语义拓扑 — 1 天

**操作**：
1. 安装 Ultralytics YOLO
2. 编写检测节点
3. 编写 `semantic_map.py`，实现 3D 投影解算

**通关验证**：
- Rviz Marker 正确显示物体 3D 坐标
- `adam_assets/maps_semantic/` 下有有效 JSON

### 🛠️ 步骤 3：Ollama 具身全链条 — 1 天

**操作**：
1. 宿主机安装 Ollama，拉取 Qwen-2.5-Instruct
2. 编写 `llm_reasoner.py`
3. 打通全链条：自然语言 → 坐标 → Nav2 Action → 底盘运动

**通关验证**：
- 全栈终极铁律：输入"去桌子那里" → 小车平滑开到桌子前 50cm

---

## 4. 依赖安装

```bash
# YOLOv11
pip install ultralytics

# Ollama（宿主机，不在 ROS2 容器内）
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen2.5:7b-instruct

# 或使用 VL 版本
ollama pull qwen2.5-vl:7b
```

---

## 5. 交付标准总览

| 单元 | 产出 | 通关标准 | 工期 |
|------|------|---------|------|
| Unit 1 | Lifecycle 状态机 | 子节点按矩阵无缝切换，无 Ghost | 1d |
| Unit 2 | YOLO + 语义拓扑 | 物体 3D 坐标正确解算，JSON 落盘 | 1d |
| Unit 3 | Ollama 任务拆解 | 自然语言 → 目标点 → 自主导航闭环 | 1d |

---

## 6. 全栈终极通关铁律

输入模糊指令"去桌子那里"：

1. ✅ Ollama 准确吐出桌子的坐标 JSON
2. ✅ 中枢控制器调度全局 EKF 和 Nav2 3D
3. ✅ 小车平滑启动，自动绕过路上临时放置的板凳
4. ✅ 最终精准停在桌子前方 50cm 处
5. ✅ 系统数据、资产、运控全线大闭环
