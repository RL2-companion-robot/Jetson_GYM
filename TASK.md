# 项目进度记录

## 问题
需要在仓库内维护一份可持续回顾的上下文文档，集中记录当前任务、背景、进度、已完成事项、未完成事项以及关键注意点，降低新开对话时对历史聊天记录的依赖。

## 背景
当前仓库是 Jetson 平台上的双足机器人强化学习部署项目，核心是基于 TensorRT 的实时推理、UDP 通信和若干硬件测试工具。此前已补充仓库级贡献说明文档 `AGENTS.md`，用于记录固定规则和开发约定。

## 当前进度
已建立仓库内的阶段性上下文文档机制：本文件 `TASK.md` 作为当前会话与后续会话衔接的统一记录入口。后续只要发生代码、文档或配置修改，都应同步更新本文件。
已完成对当前仓库源码、测试工具、转换脚本和配置文件的第一轮通读，对“文件职责”和“从模型到实机控制”的主 pipeline 已有较完整理解。
已删除仓库中与当前实现不一致的两个 Python 模型转换脚本，当前保留的模型转换入口为 C++ 工具 `tools/onnx_to_trt.cpp`。
已将 Jetson/ODroid 通信协议升级为包含 `quat[4]` 的 232 字节请求包，并在主通信路径及主要联调工具中补充长度校验；推理仍保持使用本地 `robot.yaml` 标定姿态，不使用通信中的 `init_pos`。
已增强 `test/test_udp.cpp` 的终端输出：周期性打印完整接收请求包、完整发送响应包，以及异常长度、超时、发送失败等统计信息。
已为 `test/test_udp.cpp` 添加 `--verbose-every N` 参数，用于控制打印频率；传 `1` 可逐包打印。
已将 `test/test_motors.cpp` 的初始化阶段改为 5 秒平滑插值到 `init_pose`，不再在连接后直接硬切到初始姿态。
已将 `test/test_init_pose.cpp` 的插值流程和 `test_motors.cpp` 对齐：统一使用首次有效反馈作为插值起点，未收到反馈时回退到 `init_pose` 本身，并显式写入 `dq_exp/tau_exp = 0`。
已将 `test/test_init_pose.cpp` 改为固定 5 秒定时插值；同时主部署程序 `src/main.cpp` 的 `moveToInitPose()` 也改为固定 5 秒回到 `init_pose`。
已按当前控制语义调整主程序输出限幅顺序：先将滤波后的相对动作限幅到 `[-1.57, 1.57]`，再乘缩放并叠加 `calibrated_init_pos`，不再对最终位置做 `[-5, 5]` 绝对限幅。
已在主部署程序中加入两层一次触发即回退的安全保护：`motor_cmd` 超过 `[-1.57, 1.57]` 或任一关节扭矩反馈超过 `1.5 Nm` 时，立即停止当前推理并用 5 秒插值回到 `init_pose`。
已新增异步 CSV 复盘日志链路：主程序现在会在 `data/` 下生成时间戳命名的新 CSV 文件，成功推理后记录一行 `inference`，异常保护或 `Ctrl+C` 时记录一行 `event`，由后台线程负责格式化时间戳、批量写盘并每 500ms flush 一次。
已新增 `test/test_capture_init_pose_observation.cpp`，用于在 Jetson 上保持机器人处于 `init_pose`、实时接收 UDP 状态，并导出静站阶段的 39 维策略观测到 `data/` 下的单个 CSV。
已新增 `test/test_zero_pose.cpp`，用于将 10 个关节从首次反馈位置在固定 5 秒内平滑插值到全零位，并在到达后持续保持零位。
已根据 `Yuanbao_Deploy.urdf` 中各关节的 `lower/upper` 限位，将主程序中的统一 `clip` 改为按关节分别限幅：先根据 `robot.yaml` 的 `init_pose` 计算每个关节允许的相对偏移范围，再对最终 `motor_cmd` 按各关节的绝对位置上/下限做安全保护。
已新增统一标定配置接口 `init_pose + offset`，并抽出共享读写模块 `calibration_config`。
旧手动标定工具现已输出统一格式 YAML：`init_pose` 为人工采集值，`offset` 全为 `0`；同时新增 `calibration_sim_offset`，将机器人移动到代码里写死的仿真 `init_pose`，稳定后采样编码器均值并计算 `offset = encoder_raw - sim_init_pose`。
主程序已切换到统一补偿逻辑：推理前使用 `q_obs = q_raw - offset` 作为网络观测，发送给 ODroid 的目标位置使用 `q_cmd = q_policy + offset`；所有回 `init_pose` 的控制路径也统一带上 offset。
`test_init_pose.cpp`、`test_motors.cpp`、`test_capture_init_pose_observation.cpp` 已同步改为读取统一格式 YAML，并在发送位置目标时自动加上 offset。

## 已完成项
- 新增 `AGENTS.md`，记录仓库贡献指南、目录结构、构建命令、测试方式和提交要求。
- 新增 `TASK.md`，用于记录当前阶段任务状态与交接信息。
- 明确本仓库的上下文维护方式采用“单文档持续更新”，不强制拆分成多层文档体系。
- 已梳理当前代码库中的核心模块：UDP 通信、TensorRT 推理、手柄输入、标定工具、模型转换脚本以及若干测试程序。
- 已确认主程序 `src/main.cpp` 的运行链路为：读取标定 -> 初始化 UDP -> 加载 TensorRT 引擎 -> 机器人移动到初始姿态 -> 进入 500Hz 控制循环 -> 接收状态 -> 可选手柄覆盖 command -> 推理 -> 生成电机目标位置 -> 发送回 ODroid -> 退出时导出 CSV。
- 已删除 `scripts/convert_to_trt.py` 和 `scripts/convert_to_onnx.py`，原因是它们与当前 C++ 推理接口和现行流程不一致，保留会增加误导风险。
- 已同步更新 `MsgRequest` 协议，新增 `quat[4]`，并在 `communication.cpp`、`calibration_tool.cpp`、`test_udp.cpp`、`test_init_pose.cpp`、`test_motors.cpp` 中补充接收包长度校验。
- 已扩展 `test_udp.cpp` 的调试输出，当前可查看完整请求字段、完整响应字段，以及收发统计。
- 已为 `test_udp.cpp` 添加打印频率参数，默认每 250 个有效包打印一次。
- 已调整 `test_motors.cpp` 初始化流程：从首次反馈关节位置线性插值到 `robot.yaml` 中的 `init_pose`。
- 已统一 `test_init_pose.cpp` 与 `test_motors.cpp` 的插值起点和目标保持逻辑。
- 已将 `test_init_pose.cpp` 与主程序 `moveToInitPose()` 的时间语义统一为固定 5 秒插值。
- 已修改主程序中动作到位置指令的限幅顺序和阈值，当前偏移限幅为 `[-1.57, 1.57]`。
- 已在 `main.cpp` 中增加 `motor_cmd` 越界保护和扭矩反馈超限保护，触发条件均为一次即回退。
- 已新增 `csv_logger` 模块，并将主程序的成功推理记录、异常事件记录和退出收尾接入同一个 CSV。
- 已新增静站观测采集测试程序，导出的观测使用与主程序一致的缩放规则，但固定 `command=0`、`last_action=0`，专门用于与 `init_pose` 静站仿真阶段做分布对比。
- 已新增零位测试程序 `test_zero_pose.cpp`，用于在安全联调时将各关节平滑回到 `0 rad` 并持续保持。
- 已将主程序中的相对动作限幅与最终目标位置限幅都切换为按关节分别生效，不再使用统一的 `±1.57 rad`。
- 已新增 `calibration_config` 共享模块，统一管理 `init_pose + offset` 的 YAML 读写。
- 已保留旧手动标定方式，并将其输出改为统一格式：`init_pose` 为人工采集值，`offset` 全为 0。
- 已新增 `calibration_sim_offset` 标定程序，输出 `init_pose=仿真指定值` 与 `offset=encoder_raw - sim_init_pose`。
- 已将主部署程序切换到补偿坐标系：策略内部使用补偿后的关节位置，发送给 ODroid 的目标位置则自动加回 offset。
- 已将 `test_init_pose.cpp`、`test_motors.cpp`、`test_capture_init_pose_observation.cpp` 切换到统一配置接口，避免在新 YAML 中误读 `offset`。

## 剩余未完成事项
- 后续每次实际修改代码、文档、构建配置或运行方式后，更新本文件中的“当前进度 / 已完成项 / 剩余未完成事项 / 风险和约束”。
- 当出现新的明确开发目标时，在本文件补充具体任务目标、完成标准和阻塞项。
- 若后续要继续开发，应进一步确认训练侧 ONNX 输入命名、历史观测定义以及实机 ODroid 端协议是否与当前 TensorRT 推理实现完全一致。
- 若后续要做稳定性整改，应检查 `README.md` 和实际源码之间的漂移，例如导出路径、模型输入接口和文档中的旧描述。
- 若后续仍需要 Python 侧模型导出能力，应基于当前双输入 TensorRT 接口重新实现，而不是恢复旧脚本。
- 若后续需要让更多工具支持补偿坐标系，应继续梳理 `test_trt_engine`、`test_trt_engine_csv` 等是否也要感知 offset。

## 注意点 / 细节
- 本文件放在仓库根目录，路径固定为 `TASK.md`。
- 记录应偏事实，不写冗余对话内容，重点保留下一次会话继续工作所需的信息。
- 若仓库实际结构、命令或依赖发生变化，应优先更新 `AGENTS.md`，再同步在本文件记录变更结果。
- 若存在未提交代码、临时验证结论、环境差异或硬件依赖，也应写入本文件，避免下个会话重复排查。
- 当前仓库里 `README.md` 基本正确，但和实际文件存在少量漂移，例如多了 `gamepad_input` 模块、`test_init_pose.cpp`，以及主程序当前导出路径是 `../data/inference_data.csv`。
- 当前主程序使用 `robot.yaml` 作为算法层默认站姿/初始姿态来源，并未在推理时使用 `MsgRequest.init_pos` 动态覆盖。
- 当前协议虽然携带 `quat[4]`，但现有策略观测仍使用 `eu_ang`，未将四元数接入推理输入。
- `test_udp.cpp` 现在会打印较大量的终端信息，默认每 250 个有效包打印一次；需要逐包时可传 `--verbose-every 1`。
- `test_motors.cpp` 现在的初始化段依赖首次有效反馈中的 `request.q[10]` 作为插值起点；如果首次反馈异常，初始化起点会退化为目标 `init_pose`。
- `test_init_pose.cpp` 已不再支持 `--steps`；当前仓库里回到 `init_pose` 的主逻辑统一采用固定 5 秒插值。
- 主程序当前仍保留 `trt_inference.cpp` 内部的一层动作平滑/限幅，以及 `main.cpp` 外层的一层动作平滑；本次只调整了 `main.cpp` 的限幅顺序，没有进一步重构双重滤波。
- 当前扭矩保护按单次超限立即触发，没有连续计数或去抖逻辑；如果底层扭矩反馈存在瞬时毛刺，可能会比较敏感。
- 当前按关节 `clip` 的限位值来自外部 URDF：`/home/zomnk/Documents/Yuanbao_RL_IsaacGym/resources/robots/ours_v2/URDF/Yuanbao_Deploy/urdf/Yuanbao_Deploy.urdf`；如果后续 URDF 更新，部署侧限位常量也需要同步更新。
- 当前复盘 CSV 不包含温度字段，因为 Jetson/ODroid 现有通信协议里没有 10 个电机温度。
- `event` 行默认复用最近一次完整 `inference` 快照；如果启动后尚未产生成功推理，就会退化为使用当前可用的 `request/response` 数据或零值。
- `test_capture_init_pose_observation.cpp` 不是“真实部署观测采集器”，因为它固定 `command=0` 且不运行策略，因此更适合和静站仿真阶段做对齐，而不是和策略闭环阶段做对齐。
- `test_zero_pose.cpp` 是直接回全零位的联调工具，不读取 `robot.yaml`；使用前应确认机器人机械零位和控制零位一致，避免因零位定义不一致导致姿态风险。
- 新统一格式 YAML 的语义是：`init_pose` 为网络/策略坐标系基准，`offset` 满足 `q_obs = q_raw - offset` 与 `q_cmd = q_policy + offset`。
- `calibration_sim_offset.cpp` 当前将仿真 `init_pose` 写死在代码中；如果仿真默认姿态有变动，需要同步更新该文件。
- 当前仓库默认 `--config ../robot.yaml` 仍可继续使用，因为统一读取器会把缺失的 `offset` 当作 0；新的推荐输出文件分别是 `robot_manual_calibration.yaml` 与 `robot_sim_offset_calibration.yaml`。

## 风险和约束
- 该项目依赖 Jetson、CUDA、TensorRT 和部分硬件接口，很多验证步骤无法在无设备环境下完整复现。
- TensorRT 引擎与目标设备架构绑定，模型转换和部分测试必须在目标 Jetson 上完成。
- 仓库中的 README 与实际文件可能存在轻微漂移，后续修改前仍需以当前代码和构建脚本为准。
- 当前仓库仅保留 C++ 侧 `onnx_to_trt` 转换工具；如果后续需要恢复 Python 转换链路，必须先重新确认模型输入输出签名。

## 当前建议的维护方式
- 每次完成一个可识别的修改后，立即更新本文件。
- 新开对话时，优先阅读 `TASK.md`，再按需查看 `AGENTS.md` 和相关源码文件。
