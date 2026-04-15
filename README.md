# Jetson RL 部署项目 (TensorRT版)

本项目用于在 Jetson 平台上部署双足机器人强化学习策略。Jetson 通过 UDP 与 ODroid 通信，在 500Hz 控制循环中完成观测构造、TensorRT 推理、动作后处理和电机目标位置下发。

## 项目概述

**核心功能**：
- TensorRT 推理部署
- UDP 实时通信
- 500Hz 主控制循环
- 手柄输入覆盖 command
- 统一标定接口：`init_pose + offset`
- 推理复盘 CSV 自动记录

**系统架构**：
```text
STM32 -> ODroid -> [UDP] -> Jetson (TensorRT推理) -> ODroid -> STM32
```

## 项目结构

```text
project/
├── CMakeLists.txt
├── README.md
├── robot.yaml                              # 旧格式兼容标定文件（offset 缺失时按 0 处理）
├── include/
│   ├── types.h
│   ├── communication.h
│   ├── trt_inference.h
│   ├── gamepad_input.h
│   ├── csv_logger.h
│   └── calibration_config.h               # init_pose + offset 统一配置接口
├── src/
│   ├── main.cpp
│   ├── communication.cpp
│   ├── trt_inference.cpp
│   ├── gamepad_input.cpp
│   ├── csv_logger.cpp
│   ├── calibration_config.cpp
│   ├── calibration_tool.cpp               # 旧手动标定：offset=0
│   └── calibration_sim_offset.cpp         # 仿真 init_pose 对齐标定
├── test/
│   ├── test_udp.cpp
│   ├── test_motors.cpp
│   ├── test_init_pose.cpp
│   ├── test_zero_pose.cpp
│   ├── test_capture_init_pose_observation.cpp
│   ├── test_trt_engine.cpp
│   ├── test_trt_engine_csv.cpp
│   └── gamepad_calibration.cpp
└── tools/
    └── onnx_to_trt.cpp
```

## 数据接口

### UDP 消息

- `MsgRequest`: `58` 个 `float`，共 `232 bytes`
- `MsgResponse`: `30` 个 `float`，共 `120 bytes`

`MsgRequest` 主要包含：
- `trigger`
- `command[4]`
- `eu_ang[3]`
- `omega[3]`
- `acc[3]`
- `q[10]`
- `dq[10]`
- `tau[10]`
- `init_pos[10]`（当前保留传输，Jetson 推理不使用）
- `quat[4]`

### 策略观测 (39维)

```text
omega(3) + eu_ang(3) + command(3) + (q - init_pose)(10) + dq(10) + last_action(10)
```

## 统一标定接口

当前统一使用 YAML 中的两个字段：
- `init_pose`
- `offset`

语义如下：
- `init_pose`: 网络/策略坐标系下的初始姿态
- `offset`: 编码器坐标系到网络坐标系的补偿量

运行时满足：

```text
q_obs = q_raw - offset
q_cmd = q_policy + offset
```

### 两种标定方式

1. `calibration_tool`
- 旧手动标定方式
- 逐关节人工采集 `init_pose`
- 输出统一格式 YAML
- `offset` 全为 `0`

2. `calibration_sim_offset`
- 新仿真对齐标定方式
- 将机器人插值到代码中写死的仿真 `init_pose`
- 稳定后采样编码器均值
- 计算 `offset = encoder_raw - sim_init_pose`
- 输出统一格式 YAML

## 构建

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

主要可执行文件：
- `JetsonRLDeploy`
- `calibration_tool`
- `calibration_sim_offset`
- `test_udp`
- `test_motors`
- `test_init_pose`
- `test_zero_pose`
- `test_capture_init_pose_observation`
- `test_trt_engine`
- `test_trt_engine_csv`
- `gamepad_calibration`
- `onnx_to_trt`

## 运行主程序

```bash
./JetsonRLDeploy ../model.engine --config ../robot_manual_calibration.yaml --ip 192.168.5.159 --port 10000
```

**注意**：
- `<engine_path>` 必填
- `--config` 现在也必须显式传入
- 如果不传 `--config`，主程序会直接拒绝启动
- 推荐配置文件：
  - `../robot_manual_calibration.yaml`
  - `../robot_sim_offset_calibration.yaml`

## 模型转换

当前保留的模型转换入口是 C++ 工具：

```bash
cd build
./onnx_to_trt ../model.onnx ../model.engine
```

TensorRT 引擎与 GPU 架构绑定，必须在目标 Jetson 上生成。

## 推荐使用流程

### 1. 测试 TensorRT 引擎

```bash
./test_trt_engine ../model.engine
```

### 2. 选择一种标定方式

旧手动标定：

```bash
./calibration_tool --output ../robot_manual_calibration.yaml
```

仿真对齐标定：

```bash
./calibration_sim_offset --output ../robot_sim_offset_calibration.yaml
```

### 3. 测试回姿态

```bash
./test_init_pose --ip 192.168.5.159 --config ../robot_manual_calibration.yaml
```

### 4. 运行主程序

```bash
./JetsonRLDeploy ../model.engine --config ../robot_manual_calibration.yaml
```

如果你使用的是仿真对齐标定，则建议按下面这组命令启动：

```bash
./calibration_sim_offset --output ../robot_sim_offset_calibration.yaml
./test_init_pose --ip 192.168.5.159 --config ../robot_sim_offset_calibration.yaml
./JetsonRLDeploy ../model.engine --config ../robot_sim_offset_calibration.yaml
```

## 测试程序

### UDP 通信测试

```bash
./test_udp [IP] [PORT]
```

### 电机正弦运动测试

```bash
./test_motors --ip 192.168.5.159 --config ../robot_manual_calibration.yaml
```

### 回初始姿态测试

```bash
./test_init_pose --ip 192.168.5.159 --config ../robot_manual_calibration.yaml
```

### 回零位测试

```bash
./test_zero_pose --ip 192.168.5.159 --port 10000
```

### 静站观测采集

```bash
./test_capture_init_pose_observation --ip 192.168.5.159 --config ../robot_manual_calibration.yaml
```

### TensorRT 与 CSV 对比

```bash
./test_trt_engine_csv ../model.engine ../sim_inputs_outputs_100steps.csv
```

### 手柄键码获取

```bash
./gamepad_calibration /dev/input/js0
```

## 日志与复盘

主程序运行时会在 `data/` 下自动生成时间戳命名的 CSV，例如：

```text
data/deploy_log_2026-04-15_14-30-20.csv
```

CSV 包含两类记录：
- `inference`: 成功推理后的期望位置/速度/力矩、实际位置/速度/力矩、IMU
- `event`: `Ctrl+C`、推理失败、NaN、扭矩超限、`motor_cmd` 越界等事件

## 当前控制与安全逻辑

- `moveToInitPose()` 统一使用固定 `5s` 插值
- 主程序有两层一次触发即回退保护：
  - 扭矩反馈超过 `1.5 Nm`
  - 最终 `motor_cmd` 超出关节限位
- 关节位置限位来自 URDF，当前已改为按关节分别 clip
- `calibration_sim_offset` 在开始前要求人工输入大写 `YES` 确认，并带有关节限位和扭矩保护

## 常见问题

### 1. 主程序启动时报错必须传 `--config`

这是当前设计要求。请显式传入正确的标定文件，例如：

```bash
./JetsonRLDeploy ../model.engine --config ../robot_manual_calibration.yaml
```

### 2. TensorRT 找不到

```bash
cmake .. -DTENSORRT_ROOT=/usr/local/tensorrt
```

### 3. 标定文件加载失败

- 检查 YAML 是否包含 `init_pose`
- 新格式推荐同时包含 `offset`
- 旧格式 `robot.yaml` 仍可兼容读取，此时 `offset` 自动视为 `0`

### 4. 推理结果异常

- 检查模型输入输出签名是否正确
- 检查标定文件是否与当前机器人一致
- 检查 `offset` 是否来自正确的标定流程

## 开发说明

仓库内持续进度记录见 [TASK.md](TASK.md)。  
贡献和协作约定见 [AGENTS.md](AGENTS.md)。
