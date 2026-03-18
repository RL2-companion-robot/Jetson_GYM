# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Jetson RL Deployment is a TensorRT-based inference system for real-time control of a bipedal robot. The system runs on Jetson platforms (Nano/Xavier/Orin) and communicates with an ODroid controller via UDP to execute reinforcement learning policies.

**Key Architecture**: STM32 → ODroid → UDP → Jetson (TensorRT inference) → ODroid → STM32

## Build & Compilation

```bash
cd project
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

**Generated executables:**
- `JetsonRLDeploy` - Main inference program
- `calibration_tool` - Joint calibration utility
- `gamepad_calibration` - Gamepad input testing tool
- `test_udp`, `test_motors`, `test_trt_engine` - Test utilities
- `onnx_to_trt` - Model conversion tool

## Core Architecture

### Data Flow
1. **Input**: 39-dimensional observation vector (angular velocity, Euler angles, control commands, joint positions/velocities, previous actions)
2. **Processing**: TensorRT inference engine executes the RL policy
3. **Output**: 10-dimensional action vector (motor commands for 5 joints per leg)

### Key Components

**Communication Layer** (`include/communication.h`, `src/communication.cpp`)
- UDP socket management for ODroid communication
- Message structures: `MsgRequest` (robot state) and `MsgResponse` (motor commands)
- Non-blocking receive with timeout handling

**Inference Engine** (`include/trt_inference.h`, `src/trt_inference.cpp`)
- TensorRT engine loading and execution
- Observation building from sensor data
- Action filtering (0.8 * current + 0.2 * previous)
- Motor command scaling: `motor_cmd = filtered_action * 0.25 + init_pos`

**Main Control Loop** (`src/main.cpp`)
- 500Hz control frequency (2ms per cycle)
- Graceful shutdown on Ctrl+C with CSV data export
- Automatic recovery to init pose on inference failure or NaN detection
- Real-time data recording with timestamps

### Message Structures (`include/types.h`)
- `MsgRequest`: Contains joint positions (q[10]), velocities, IMU data, control commands
- `MsgResponse`: Contains expected joint positions (q_exp[10])
- `ACTION_DIM = 10`, `OBS_DIM = 39`

## Configuration & Calibration

**Calibration File** (`robot.yaml`)
- YAML format with nested structure: `robot_config.init_pose.{left_leg,right_leg}.{yaw,roll,pitch,knee,ankle}`
- 10 float values representing initial joint positions in radians
- Loaded by `loadCalibration()` function which parses the nested YAML structure

**Calibration Tool** (`src/calibration_tool.cpp`)
- Interactive joint-by-joint calibration
- Outputs to `robot.yaml` with timestamp

## Recent Enhancements

### Gamepad Input Support
- New tool: `gamepad_calibration` for reading gamepad events from `/dev/input/event*`
- Supports both joystick axes and button inputs
- Non-blocking event reading with diagnostics

### CSV Data Export
- Inference data automatically exported to `inference_data.csv` on program exit
- Records: timestamp (seconds since start), 10 motor commands per row
- Useful for post-analysis and debugging

### YAML Parsing Fix
- `loadCalibration()` now correctly handles nested YAML structures
- Skips configuration labels (left_leg, right_leg) and extracts only numeric values
- Provides detailed error messages if calibration file is malformed

## Common Development Tasks

### Testing TensorRT Engine
```bash
cd build
./test_trt_engine ../model.engine
```
Verifies CUDA availability, engine loading, and inference performance.

### Testing UDP Communication
```bash
./test_udp [IP] [port]
```

### Running Main Inference
```bash
./JetsonRLDeploy ../model.engine --config ../robot.yaml --ip 192.168.5.159 --port 10000
```

### Debugging Gamepad Input
```bash
./gamepad_calibration /dev/input/event2
```
Shows real-time axis and button events with codes.

## Important Implementation Details

### Action Processing Pipeline
1. Raw inference output → Apply exponential moving average filter
2. Filtered action → Scale by 0.25 and add init_pos offset
3. Clamp to [-5.0, 5.0] range
4. Send as motor command

### Error Handling
- NaN detection in inference output triggers automatic recovery
- Failed inference (trigger != 1.0) returns to init pose
- UDP receive timeout handled gracefully
- Signal handlers (SIGINT, SIGTERM) enable clean shutdown

### Performance Characteristics
- TensorRT inference: ~0.15ms (6666 FPS on Jetson Nano)
- Control loop: 500Hz (2ms per cycle)
- CSV export: Minimal overhead, recorded in-memory during execution

## File Organization

```
project/
├── include/
│   ├── types.h              # Message structures (MsgRequest, MsgResponse)
│   ├── communication.h      # UDP communication class
│   └── trt_inference.h      # TensorRT inference class
├── src/
│   ├── main.cpp             # Main control loop with CSV export
│   ├── communication.cpp    # UDP implementation
│   ├── trt_inference.cpp    # TensorRT engine management
│   └── calibration_tool.cpp # Joint calibration utility
├── test/
│   ├── gamepad_calibration.cpp  # Gamepad input testing
│   ├── test_trt_engine.cpp      # Engine verification
│   ├── test_udp.cpp             # UDP communication test
│   ├── test_motors.cpp          # Motor control test
│   └── test_init_pose.cpp       # Init pose movement test
├── tools/
│   └── onnx_to_trt.cpp      # Model conversion utility
├── robot.yaml               # Calibration configuration
└── CMakeLists.txt           # Build configuration
```

## Key Constants & Limits

- `ACTION_DIM = 10` (5 joints per leg)
- `OBS_DIM = 39` (3 angular velocity + 3 Euler + 3 control + 10 positions + 10 velocities + 10 previous actions)
- Motor command range: [-5.0, 5.0]
- Control frequency: 500Hz (2ms cycle)
- Action filter coefficient: 0.8 (current) / 0.2 (previous)
- Action scaling factor: 0.25

## Debugging Tips

1. **Inference failures**: Check for NaN in output, verify model input/output dimensions match OBS_DIM/ACTION_DIM
2. **Calibration issues**: Run `gamepad_calibration` to verify hardware, check `robot.yaml` format
3. **UDP communication**: Use `test_udp` to verify network connectivity
4. **Performance**: Check TensorRT engine generation was done on target device (engines are GPU-architecture specific)
5. **CSV data**: Inspect `inference_data.csv` for action patterns and timing anomalies

## 🌐 通用开发规则

【语言规则】
- 始终用中文回答

【修改规则】
- -Always use Context7 MCP when I need library/API documentation, code generation, setup or configuration steps without me having to explicitly ask.
- 每次修改都必须先全链条理解清楚所有有关联的上下文代码和需求后再改
- 只做最少修改，完成需求
- 修改完后自行检查代码，确保能编译通过

【禁止操作】
- 禁止修改任何与需求无关的功能
- 禁止删除.csv/.xlsx文件

## ⚡ 无限的上下文窗口

【自动摘要启用】
- 通过自动摘要实现无限的上下文窗口
