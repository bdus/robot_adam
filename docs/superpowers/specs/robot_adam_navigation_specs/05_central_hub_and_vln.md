# 机器人中枢控制器与大模型语义决策详细规格说明书 (SPEC 05_Hub_VLN)

> **版本**: v1.0 | **日期**: 2026-06-28 | **状态**: Working
> **存放目录**: `docs/superpowers/specs/robot_adam_navigation_specs/05_central_hub_and_vln.md`
> **范围**：`adam_hub_controller` Lifecycle 状态机总控 + YOLO-World 开放域 3D 射线投影 + 语义拓扑地图字典持久化 + 本地 Ollama 自然语言智能体任务树解析与 Nav2 驱动。
> **关联总设计**：[`docs/spec/01_robot_adam_navigation_architecture.md`](../../../spec/01_robot_adam_navigation_architecture.md) — 本系列宏观架构纲领。
> **前置依赖**：`SPEC 01` 到 `SPEC 04` 全部高分通过通关测试（2D/3D 激光、STVL 避障、多模态视觉感知已完全单点筑基就绪）。
> **总工期预估**：5 天 | **原子交付单元数**：4

---

## 1. 本期范围与边界 (Scope & Boundary)

### 1.1 本期开发目标

作为全栈系统的最后一环，本期目标是彻底激活机器人的"具身智能完全体"。中枢控制器利用 ROS2 标准的 `LifecycleNode` 状态机接管下方所有算法节点的软启动、挂起与消亡。高层消费 RGB-D 图像进行零样本（Zero-Shot）物体三维反投影，动态沉淀并更新语义拓扑文件。最终，通过本地化大模型解析模糊的人类语音指令，转化为标准任务动作树，调用下方筑基好的 Nav2 运控底座四平八稳地抵达目标语义点。

### 1.2 本期严格不包含

本期为收官阶段，所有涉及底层 SLAM、里程计融合、TF 树发布、局部规划避障的底层算法**仅做生命周期受控挂载，严格禁止在控制中枢中编写任何具体算法的数学解算代码**。

---

## 2. 最小原子交付单元拆解 (Sub-Agent Units)

### 📦 Unit 1: adam_hub_controller 生命周期状态机托管中枢

**功能描述**：基于标准 `rclcpp_lifecycle` 编写核心总控节点。它不参与任何运动学计算，但死死管住底层各路算法（Cartographer、explore_lite、Nav2、FAST-LIO2）的启动与死活，根除因多算法竞态冲突导致的 TF 树穿孔。

**业务状态迁移矩阵规格**：

提供一个标准的控制服务接口 `/adam_hub/switch_mode`（自定义服务类型，含 `target_mode` 字段）：

- **Mode 1: `MAPPING_EXPLORE`（探索建图模式）**：中枢驱动底层 Lifecycle 子节点跳转，触发 FAST-LIO2 和 explore_lite 进入 `Active` 状态。
- **Mode 2: `KNOWN_MAP_NAV`（已知图导航模式）**：中枢强行将 explore_lite 切换为 `Unconfigured`（彻底杀死释放算力），劫持 Cartographer 的服务将其一键切换为纯定位参数配置，并将 Nav2 状态动态唤醒至 `Active` 状态。

**状态机核心逻辑**：
```
[Unconfigured] (系统冷启动)
    │
    ▼ on_configure()
[Inactive] (中枢就绪，静默等待指令)
    │
    ├──► on_activate() [MAPPING_EXPLORE 模式]
    │      激活: cartographer_mapping, explore_lite, fast_lio2
    │      挂起: cartographer_localization, nav2
    │
    ├──► on_activate() [KNOWN_MAP_NAV 模式]
    │      激活: cartographer_localization, nav2
    │      挂起: cartographer_mapping, explore_lite
    │
    └──► on_activate() [3D_AVOIDANCE 模式]
           激活: fast_lio2, stvl_costmap, nav2_3d
           挂起: cartographer_mapping
```

**服务接口**：
| 接口 | 类型 | 说明 |
|------|------|------|
| `/adam_hub/switch_mode` | `std_srvs/srv/SetBool` | `True`=探索建图, `False`=已知图导航 |
| `/adam_hub/save_current_map` | `std_srvs/srv/Trigger` | 触发地图归档至 `adam_assets` |
| `/adam_hub/sensor_status` | Topic | 发布各传感器健康度状态 |

---

### 📦 Unit 2: YOLO-World 开放域语义 3D 射线反投影器

**功能描述**：挂载 YOLOv11-World 开放域检测网络。通过文本 Prompt 字典（如 `["refrigerator", "water_dispenser", "desk"]`）实时框出 2D 像素矩形。提取边界框中心像素 $(u, v)$ 与对应的 RGB-D 深度值 $d$，结合相机内参矩阵 $K$ 计算出相机坐标系下的 3D 坐标 $P_c$：

$$P_c = \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} = \begin{bmatrix} (u - c_x) \cdot d / f_x \\ (v - c_y) \cdot d / f_y \\ d \end{bmatrix}$$

再消费 Level 2 当前唯一的、绝对稳定的全局 TF 树（来自 `global_ekf_node` 发布的 `map -> camera_link`），将 $P_c$ 转换映射为全局 `map` 坐标系下的绝对三维物理坐标 $P_w = [X_w, Y_w, Z_w]$。

**输入依赖**：
- `/camera/image_raw`：图像话题
- `/camera/depth`：深度话题（或 Mid360 雷达点云）
- `map -> base_link` TF（来自 `global_ekf_node`）

**产出物**：
- `config/yolo_params.yaml`：YOLO 模型配置（模型权重路径、置信度阈值、类别过滤）
- `launch/yolo_detector.launch.py`：启动脚本

**3D 投影算法流程**：
```
1. YOLO 检测到物体 2D 边界框
2. 取边界框中心点像素坐标 (u, v)
3. 查询深度图/点云，获取该像素的深度值 d
4. 利用相机内参矩阵 K，将 (u, v, d) 转换为相机坐标系 3D 点 P_cam
5. 利用 TF tree (camera_link → base_link → map)，将 P_cam 转换到 map 坐标系
6. 得到物体在 map 下的绝对坐标 (x, y, z)
```

---

### 📦 Unit 3: 语义拓扑字典持久化落盘机制

**功能描述**：写一个轻量级语义沉淀脚本 `semantic_map_manager.py`。接收 Unit 2 反投影出的物体物理坐标，进行高频空间欧氏距离聚类滤波（防止同个物体被多帧噪声识别为多个不同地标）。

**资产沉淀规格**：

将去噪后的地标动态写入 `adam_assets` 资产库下的标准 JSON 字典：`adam_assets/share/maps_semantic/bigH_world_topology.json`。

**JSON 数据格式硬性规范**：
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

**服务接口**：
| 接口 | 类型 | 说明 |
|------|------|------|
| `/semantic_map/get_object` | 自定义 Service | 按标签查询物体坐标 |
| `/semantic_map/update_object` | 自定义 Service | 更新/添加物体坐标 |
| `/semantic_map/save_to_disk` | `std_srvs/srv/Trigger` | 强制持久化至 `adam_assets/maps_semantic/` |

---

### 📦 Unit 4: Ollama 本地具身大脑 Prompt 解析适配器与 Action 任务链

**功能描述**：在边缘宿主机本地部署推理框架 Ollama，加载轻量化指令模型 Qwen-2.5-Instruct。编写适配器节点 `vln_server_node.py`。

**数据流闭环设计**：

人类发布模糊口令（如："我有点渴，去冰箱那里"）→ 适配器捕获并组装系统 Prompt（将当前的 `bigH_world_topology.json` 内容作为上下文喂给大模型）→ 大模型解析语义，自动完成地标名字匹配与推理，在输出端**无条件吐出标准的、不带格式解释的纯 JSON 任务树**：

```json
{
  "intent": "navigate_to_object",
  "target": "refrigerator",
  "target_coordinates": [1.5, 2.3, 0.0],
  "fallback": "如果冰箱不在视野中，搜索最近相似物体"
}
```

→ 适配器直接解析此 JSON，提取坐标，转化为标准 ROS2 接口，高频调用 Nav2 的 `NavigateToPose` Action，驱使物理底座四平八稳地开到目的地。

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

---

## 3. Bottom-Up 路线图与单点测试验收方案 (Gate Criteria)

测试必须在仿真/真实物理环境中严格按照以下步骤推进，上一步骤未达成通关标准，严禁向前演进。

```
[步骤 1: Lifecycle 状态机转换与竞态冲突消除验收]
                       │
                       ▼
[步骤 2: YOLO 开放域物体 3D 投影与拓扑 JSON 序列化落盘验收]
                       │
                       ▼
[步骤 3: ★ 终极具身大闭环 —— 大模型模糊指令全栈跑通铁律验收 ★]
```

---

### 🛠️ 步骤 1：Lifecycle 状态机转换与子节点竞态冲突消除测试

**操作方法**：一键启动控制中枢 `adam_hub_controller`。在终端中通过服务指令调用 `/adam_hub/switch_mode`，使系统在 `MAPPING_EXPLORE`（探索建图）与 `KNOWN_MAP_NAV`（已知图导航）模式之间进行**连续 50 次的高频循环切换**。

**验收方案与白盒标准（Gate 1）**：

1. **状态机跳转验证**：使用 `ros2 lifecycle list` 检查，每次模式切换时，涉及的子节点必须能平滑、无死锁地执行 `Configuring` → `Activating` → `Deactivating` 的状态迁移。
2. **★ 零 Ghost 进程冲突铁律 ★**：切换到已知图导航模式时，`explore_lite` 必须确保被 100% 挂起或销毁，`/cmd_vel` 的控制权被完全收拢到 Nav2 控制器手中。**在连续 50 次高频模式劫持测试中，整个系统的 TF 树绝不允许发生任何短暂的一帧穿孔、重叠或抛出 "Transform timeout" 指针异常，底层里程计数据流稳如磐石**。

---

### 🛠️ 步骤 2：YOLO-World 开放域物体 3D 投影与拓扑 JSON 序列化落盘验收

**操作方法**：控制小车在仿真大厅中匀速行驶，在视野前方路过一辆仿真自行车（或一台冰箱）。让小车在行驶过程中对目标进行动态多帧侧视观测。随后，控制小车关机，检查 `adam_assets` 本地源码路径。

**验收方案与白盒标准（Gate 2）**：

1. **开放域识别验证**：在 YOLO-World 字典中随意更改文本标签（例如改为 "icebox"），小车摄像头看到冰箱时必须依然能成功框出，提取 2D 像素中心点。
2. **3D 投影与聚类精度验证**：读取反投影输出的 `/semantic_landmark_pose`。由于车子处于动态行驶中，多帧识别出的物理坐标会受噪点影响轻微抖动。聚类滤波器必须能将其强行归一化为同一个 landmark 物体。
3. **通关铁律**：打开 `src/3.navigation_ai/adam_assets/share/maps_semantic/bigH_world_topology.json` 文件，里面必须成功序列化写入了 `["refrigerator"]` 的物理位姿。且**反投影结算出的 $[X_w, Y_w]$ 全局绝对坐标，与该冰箱在 Gazebo 仿真世界中的真实物理位姿金标准（Ground Truth）对比，空间绝对误差必须 ≤15 厘米**。

---

### 🛠️ 步骤 3：★ 终极具身大闭环 —— 大模型模糊指令全栈跑通铁律测试 ★

**操作方法**：这是 Robot Adam 全工程的终极通关合流测试。小车开机，中枢控制其进入已知图导航状态。人为拔掉键盘、鼠标与操作手柄，将小车随机放置在远离地标 10 米开外的初始未知点。在终端中对机器人具身大脑发送一句**人类极其模糊、不包含具体坐标数字、带有强烈口语化属性的自然语言指令**：

> *"Adam，我现在有点累了，我想去那个用来放食物保鲜的巨大白盒子冰箱那儿休息一下，你把我开过去，顺便帮我绕开路上的各种破铜烂铁障碍物。"*

**验收方案与全栈终极通关标准（Gate 3 - 具身完全体释放）**：

1. **大脑大语言模型推理验证**：本地 Ollama 必须准确在 **1.5 秒内**完成语义泛化和实体消歧，准确识别出"用来放食物保鲜的巨大白盒子"指的是资产库中的 `refrigerator`。
2. **任务树下发时效验证**：适配器成功读取 JSON 目标坐标并下发给 Nav2 Action，Action 状态瞬间跳变为 `ACTIVE`。
3. **立体避障与底层稳健筑基验证**：小车平滑启动。在全速驶向冰箱的必经之路上，人工恶意在小车正前方丢下一个**极其复杂的悬空板凳（SPEC 03 筑基）**。Nav2 MPPI 控制器在前向时空推演中敏锐捕捉到 3D 体素代价升高，底盘以极其优美的运动学弧线丝滑绕过板凳（减速不超过 20%）。
4. **★ 终极通关铁律 ★**：在导航进行到第 5 秒时，人工突然**将整个环境的光照全部关闭、使其陷入漆黑一片的瞬时盲目状态（SPEC 04 筑基）**。全局 EKF 监控节点在 10 毫秒内触发卡方检验拒绝，强行劫持并断开视觉定位流。
5. **终极闭环效果**：在全黑暗室中，底座依靠 3D 雷达和常驻轮速卡尔曼滤波器疯狂托底续命，**整车完全没有发生任何由于视觉致盲产生的位置跃迁、剧烈抖动或惊慌急刹。小车保持稳健的运控姿态在黑夜中继续向前滑行，最终四平八稳地滑行至冰箱前方 50 厘米处，优雅刹停，Action 返回成功日志 SUCCESS**。

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

| 单元 | 产出物 | 通关标准 | 工期 |
|------|-------|---------|------|
| Unit 1 | Lifecycle 状态机（adam_hub_controller） | 50 次高频切换零 Ghost 进程，TF 树无穿孔 | 1.5d |
| Unit 2 | YOLO-World + 3D 射线反投影 | 开放域标签零样本检测，3D 坐标 Rviz Marker 正确显示 | 1d |
| Unit 3 | semantic_map_manager.py + JSON 拓扑字典 | 聚类去噪，反投影误差 ≤15cm，JSON 落盘可检索 | 1d |
| Unit 4 | Ollama Prompt 适配器 + Nav2 Action 驱动 | 1.5s LLM 推理，悬空板凳绕行，关灯不减速，冰箱前 50cm 刹停 | 1.5d |
