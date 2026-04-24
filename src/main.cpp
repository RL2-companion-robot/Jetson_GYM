/**
 * @file main.cpp
 * @brief 主程序入口文件
 * @author Zomnk
 * @date 2026-02-04
 *
 * @details 本文件是Jetson RL部署程序的主入口。
 *          主要功能：
 *          1. 解析命令行参数
 *          2. 加载标定配置文件
 *          3. 初始化UDP通信
 *          4. 加载TensorRT引擎
 *          5. 运行500Hz控制循环
 *
 * @note 使用方法:
 *       ./JetsonRLDeploy <engine_path> [--ip IP] [--port PORT] [--config FILE]
 */

#include "calibration_config.h"
#include "csv_logger.h"
#include "communication.h"
#include "trt_inference.h"
#include "gamepad_input.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <csignal>
#include <cmath>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <array>
#include <cerrno>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <utility>

/*
 * ============================================================
 * 全局变量
 * ============================================================
 */

/// 运行标志，用于控制主循环
/// 当收到SIGINT或SIGTERM信号时设为false
volatile bool g_running = true;

constexpr float kActionToPositionScale = 0.25f;
constexpr float kTorqueFeedbackLimitNm = 10.0f;
constexpr uint64_t kEulerBiasSettleUs = 300000ULL;
constexpr uint64_t kEulerBiasSampleUs = 1000000ULL;
constexpr int kEulerBiasMinSamples = 100;

constexpr std::array<const char*, ACTION_DIM> kJointNames = {{
    "joint_l_yaw",
    "joint_l_roll",
    "joint_l_pitch",
    "joint_l_knee",
    "joint_l_ankle",
    "joint_r_yaw",
    "joint_r_roll",
    "joint_r_pitch",
    "joint_r_knee",
    "joint_r_ankle"
}};

// 关节绝对位置限位来自：
// /home/zomnk/Documents/Yuanbao_RL_IsaacGym/resources/robots/ours_v2/URDF/Yuanbao_Deploy/urdf/Yuanbao_Deploy.urdf
constexpr std::array<float, ACTION_DIM> kJointPosLowerLimits = {{
    -0.20f, -0.09f, -1.57f, -1.57f, -1.35f,
    -0.76f, -0.09f, -1.57f, -1.57f, -1.35f
}};

constexpr std::array<float, ACTION_DIM> kJointPosUpperLimits = {{
     0.76f,  0.56f,  1.57f,  1.57f,  1.57f,
     0.20f,  0.56f,  1.57f,  1.57f,  1.57f
}};

/**
 * @brief 信号处理函数
 *
 * @details 处理SIGINT(Ctrl+C)和SIGTERM信号，
 *          设置g_running为false以优雅退出主循环。
 *
 * @param sig 信号编号
 */
void signalHandler(int sig) {
    std::cout << "\n收到信号 " << sig << ", 正在关闭..." << std::endl;
    g_running = false;
}

bool ensureDirectoryExists(const std::string& path) {
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }

    if (mkdir(path.c_str(), 0775) == 0) {
        return true;
    }

    if (errno == EEXIST) {
        return true;
    }

    std::cerr << "无法创建目录: " << path << std::endl;
    return false;
}

std::string buildCsvLogPath() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm;
    localtime_r(&now_time, &local_tm);

    std::ostringstream oss;
    oss << "data/deploy_log_" << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
    return oss.str();
}

/**
 * @brief 打印使用说明
 *
 * @param prog 程序名称（argv[0]）
 */
void printUsage(const char* prog) {
    std::cout << "用法: " << prog << " <engine_path> [选项]" << std::endl;
    std::cout << "  engine_path: TensorRT引擎文件路径 (.engine)" << std::endl;
    std::cout << "  --ip <IP>:    ODroid IP地址 (默认: 192.168.5.159)" << std::endl;
    std::cout << "  --port <N>:   UDP端口 (默认: 10000)" << std::endl;
    std::cout << "  --config <F>: 标定文件路径 (必填，推荐: ../robot_manual_calibration.yaml)" << std::endl;
}

/**
 * @brief 缓慢移动机器人到初始姿态
 *
 * @details 使用线性插值将机器人从当前位置平滑移动到标定的初始姿态。
 *          这是一个安全措施，避免机器人突然跳到目标位置。
 *
 *          插值公式: pos = current + alpha * (target - current)
 *          其中alpha从0线性增加到1
 *
 * @param udp UDP通信对象
 * @param init_pos 目标初始姿态（网络坐标系）
 * @param offset 编码器补偿量
 */
void moveToInitPose(UDPCommunication& udp, const float init_pos[10], const float offset[10]) {
    std::cout << "正在移动到标定的初始姿态..." << std::endl;
    constexpr uint64_t kInterpolationDurationUs = 5000000ULL;  // 固定5秒插值

    MsgRequest request;
    MsgResponse response;
    std::memset(&response, 0, sizeof(response));

    // 当前位置（从反馈获取）
    float current_pos[10] = {0};
    bool got_feedback = false;

    // ========== 步骤1: 获取当前位置 ==========
    // 尝试最多50次（约500ms）获取当前关节位置
    for (int i = 0; i < 50 && !got_feedback; i++) {
        udp.sendResponse(response);
        if (udp.receiveRequest(request)) {
            // 保存当前关节位置
            for (int j = 0; j < 10; j++) {
                current_pos[j] = request.q[j] - offset[j];
            }
            got_feedback = true;
        }
        usleep(10000);  // 10ms
    }

    if (!got_feedback) {
        std::cout << "未收到反馈，使用 init_pose 作为插值起点" << std::endl;
        for (int i = 0; i < 10; i++) {
            current_pos[i] = init_pos[i];
        }
    }

    std::cout << "用5秒时间平滑插值到初始姿态..." << std::endl;

    // ========== 步骤2: 固定5秒线性插值移动 ==========
    auto interp_start = std::chrono::steady_clock::now();
    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        uint64_t elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - interp_start).count();

        float alpha = static_cast<float>(elapsed_us) / static_cast<float>(kInterpolationDurationUs);
        if (alpha > 1.0f) alpha = 1.0f;

        // 计算插值位置
        for (int i = 0; i < 10; i++) {
            float policy_q = current_pos[i] + alpha * (init_pos[i] - current_pos[i]);
            response.q_exp[i] = policy_q + offset[i];
            response.dq_exp[i] = 0.0f;
            response.tau_exp[i] = 0.0f;
        }

        // 发送位置指令
        udp.sendResponse(response);
        udp.receiveRequest(request);

        if (elapsed_us >= kInterpolationDurationUs) {
            break;
        }

        usleep(2000);  // 2ms，500Hz
    }

    for (int i = 0; i < 10; i++) {
        response.q_exp[i] = init_pos[i] + offset[i];
        response.dq_exp[i] = 0.0f;
        response.tau_exp[i] = 0.0f;
    }
    udp.sendResponse(response);

    std::cout << "已到达初始姿态" << std::endl;
}

bool calibrateInitEulerBias(UDPCommunication& udp,
                            const float init_pos[10],
                            const float offset[10],
                            float euler_bias[3]) {
    std::cout << "开始采集 init_pose 下的欧拉角零偏..." << std::endl;

    MsgRequest request;
    MsgResponse response;
    std::memset(&response, 0, sizeof(response));

    for (int i = 0; i < DOF_NUM; ++i) {
        response.q_exp[i] = init_pos[i] + offset[i];
        response.dq_exp[i] = 0.0f;
        response.tau_exp[i] = 0.0f;
    }

    float euler_sum[3] = {0.0f, 0.0f, 0.0f};
    int sample_count = 0;
    auto start = std::chrono::steady_clock::now();

    while (g_running) {
        udp.sendResponse(response);
        if (udp.receiveRequest(request)) {
            auto now = std::chrono::steady_clock::now();
            uint64_t elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();

            if (elapsed_us >= kEulerBiasSettleUs) {
                for (int i = 0; i < 3; ++i) {
                    euler_sum[i] += request.eu_ang[i];
                }
                sample_count++;
            }

            if (elapsed_us >= (kEulerBiasSettleUs + kEulerBiasSampleUs)) {
                break;
            }
        }

        usleep(2000);
    }

    if (sample_count < kEulerBiasMinSamples) {
        std::cerr << "欧拉角零偏采样失败，有效样本数不足: " << sample_count << std::endl;
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        euler_bias[i] = euler_sum[i] / static_cast<float>(sample_count);
    }

    std::cout << "欧拉角零偏采样完成: ["
              << euler_bias[0] << ", "
              << euler_bias[1] << ", "
              << euler_bias[2] << "]" << std::endl;
    return true;
}

/**
 * @brief 主函数
 *
 * @details 程序执行流程：
 *          1. 解析命令行参数
 *          2. 注册信号处理函数
 *          3. 加载标定配置
 *          4. 初始化UDP通信
 *          5. 加载TensorRT引擎
 *          6. 移动到初始姿态
 *          7. 进入500Hz控制循环
 *          8. 收到退出信号后清理退出
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 正常退出返回0，错误返回1
 */
int main(int argc, char** argv) {
    // ========== 检查命令行参数 ==========
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    // ========== 解析命令行参数 ==========
    std::string engine_path = argv[1];              // TensorRT引擎路径
    std::string target_ip = "192.168.5.159";        // ODroid IP地址
    int port = 10000;                               // UDP端口
    std::string config_file = "../robot_manual_calibration.yaml";  // 推荐标定文件路径
    bool has_config = false;

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
            has_config = true;
        }
    }

    if (!has_config) {
        std::cerr << "错误: 当前部署程序要求显式传入 --config。" << std::endl;
        std::cerr << "推荐使用: --config ../robot_manual_calibration.yaml" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    // ========== 注册信号处理函数 ==========
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signalHandler;
    sa.sa_flags = 0;  // 不使用 SA_RESTART，允许阻塞调用被中断
    sigaction(SIGINT, &sa, nullptr);   // Ctrl+C
    sigaction(SIGTERM, &sa, nullptr);  // kill命令

    // ========== 打印启动信息 ==========
    std::cout << "========================================" << std::endl;
    std::cout << "  Jetson RL 部署程序 (TensorRT)" << std::endl;
    std::cout << "  引擎: " << engine_path << std::endl;
    std::cout << "  目标: " << target_ip << ":" << port << std::endl;
    std::cout << "========================================" << std::endl;

    // ========== 加载标定配置 ==========
    CalibrationConfig calibration;
    if (loadCalibrationConfig(config_file, calibration)) {
        std::cout << "已加载标定文件: " << config_file << std::endl;
    } else {
        std::cout << "未找到标定文件，使用默认值" << std::endl;
        setZeroCalibrationConfig(calibration);
    }

    std::array<float, ACTION_DIM> filtered_action_lower_limits;
    std::array<float, ACTION_DIM> filtered_action_upper_limits;
    for (int i = 0; i < ACTION_DIM; ++i) {
        filtered_action_lower_limits[i] =
            (kJointPosLowerLimits[i] - calibration.init_pose[i]) / kActionToPositionScale;
        filtered_action_upper_limits[i] =
            (kJointPosUpperLimits[i] - calibration.init_pose[i]) / kActionToPositionScale;
    }

    std::cout << "已加载按关节位置限位:" << std::endl;
    for (int i = 0; i < ACTION_DIM; ++i) {
        std::cout << "  [" << i << "] " << kJointNames[i]
                  << " abs=[" << kJointPosLowerLimits[i] << ", " << kJointPosUpperLimits[i] << "]"
                  << " rel=[" << filtered_action_lower_limits[i] << ", " << filtered_action_upper_limits[i] << "]"
                  << std::endl;
    }
    std::cout << "已加载 offset: [";
    for (int i = 0; i < ACTION_DIM; ++i) {
        std::cout << calibration.offset[i];
        if (i + 1 < ACTION_DIM) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    // ========== 初始化UDP通信 ==========
    UDPCommunication udp(target_ip, port);
    if (!udp.init()) {
        std::cerr << "UDP初始化失败" << std::endl;
        return 1;
    }

    // ========== 加载TensorRT引擎 ==========
    TRTInference inference;
    if (!inference.loadEngine(engine_path)) {
        std::cerr << "TensorRT引擎加载失败" << std::endl;
        return 1;
    }

    if (!ensureDirectoryExists("data")) {
        return 1;
    }

    AsyncCsvLogger csv_logger;
    const std::string csv_log_path = buildCsvLogPath();
    if (!csv_logger.start(csv_log_path, std::chrono::milliseconds(500))) {
        std::cerr << "无法启动CSV日志记录器: " << csv_log_path << std::endl;
        return 1;
    }
    std::cout << "CSV复盘日志: " << csv_log_path << std::endl;

    // ========== 设置初始姿态并移动 ==========
    inference.setInitPose(calibration.init_pose);
    moveToInitPose(udp, calibration.init_pose, calibration.offset);

    // ========== 初始化手柄输入 ==========
    GamepadInput gamepad;
    if (gamepad.init("/dev/input/js0")) {
        std::cout << "手柄已连接，启用手柄控制: /dev/input/js0" << std::endl;
    } else {
        std::cout << "手柄未连接，继续使用 UDP command" << std::endl;
    }

    // ========== 开始控制循环 ==========
    std::cout << "========================================" << std::endl;
    std::cout << "开始控制循环 (500Hz)..." << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    std::cout << "========================================" << std::endl;

    MsgRequest request;
    MsgResponse response;
    std::memset(&response, 0, sizeof(response));

    // ========== 问题1: 用init_pos初始化response ==========
    // 避免第一个数据包发送零位置指令
    for (int i = 0; i < ACTION_DIM; ++i) {
        response.q_exp[i] = calibration.init_pose[i] + calibration.offset[i];
    }

    int loop_count = 0;   // 循环计数
    int infer_count = 0;  // 推理计数
    float last_action[ACTION_DIM] = {0};  // 上一步的action，用于滤波
    float euler_bias[3] = {0.0f, 0.0f, 0.0f};
    CsvLogRecord last_complete_record;
    bool has_last_complete_record = false;

    auto buildRecordFromState = [&](const MsgRequest* req_raw,
                                    const MsgRequest* req_policy,
                                    const MsgResponse* resp,
                                    const std::string& record_type,
                                    const std::string& event_type,
                                    const std::string& event_msg) {
        CsvLogRecord record;
        record.timestamp = std::chrono::system_clock::now();
        record.record_type = record_type;
        record.event_type = event_type;
        record.event_msg = event_msg;

        if (resp != nullptr) {
            for (int i = 0; i < ACTION_DIM; ++i) {
                record.q_exp[i] = resp->q_exp[i];
                record.dq_exp[i] = resp->dq_exp[i];
                record.tau_exp[i] = resp->tau_exp[i];
            }
        }

        if (req_raw != nullptr) {
            for (int i = 0; i < ACTION_DIM; ++i) {
                record.q[i] = req_raw->q[i];
                record.dq[i] = req_raw->dq[i];
                record.tau[i] = req_raw->tau[i];
            }
            for (int i = 0; i < 3; ++i) {
                record.omega[i] = req_raw->omega[i];
                record.acc[i] = req_raw->acc[i];
                record.eu_ang_raw[i] = req_raw->eu_ang[i];
            }
            for (int i = 0; i < 4; ++i) {
                record.quat[i] = req_raw->quat[i];
            }
        }

        if (req_policy != nullptr) {
            for (int i = 0; i < 3; ++i) {
                record.eu_ang[i] = req_policy->eu_ang[i];
            }
        } else if (req_raw != nullptr) {
            for (int i = 0; i < 3; ++i) {
                record.eu_ang[i] = req_raw->eu_ang[i];
            }
        }

        return record;
    };

    auto enqueueEventRecord = [&](const std::string& event_type,
                                  const std::string& event_msg,
                                  const MsgRequest* req_raw,
                                  const MsgRequest* req_policy,
                                  const MsgResponse* resp) {
        CsvLogRecord record;
        if (has_last_complete_record) {
            record = last_complete_record;
            record.timestamp = std::chrono::system_clock::now();
            record.record_type = "event";
            record.event_type = event_type;
            record.event_msg = event_msg;
        } else {
            record = buildRecordFromState(req_raw, req_policy, resp, "event", event_type, event_msg);
        }
        csv_logger.enqueue(std::move(record));
    };

    if (!calibrateInitEulerBias(udp, calibration.init_pose, calibration.offset, euler_bias)) {
        std::cerr << "初始欧拉角零偏采样失败，退出部署程序" << std::endl;
        csv_logger.stop();
        gamepad.close();
        return 1;
    }

    // ========== 主控制循环 ==========
    while (g_running) {
        gamepad.update();

        auto resetToInitPose = [&](const std::string& reason) {
            std::cout << "\n[安全保护] " << reason << std::endl;
            std::cout << "正在回到初始姿态..." << std::endl;
            inference.reset();
            std::fill(last_action, last_action + ACTION_DIM, 0.0f);
            std::memset(&response, 0, sizeof(response));
            for (int i = 0; i < ACTION_DIM; ++i) {
                response.q_exp[i] = calibration.init_pose[i] + calibration.offset[i];
            }
            moveToInitPose(udp, calibration.init_pose, calibration.offset);
            if (!calibrateInitEulerBias(udp, calibration.init_pose, calibration.offset, euler_bias)) {
                std::cerr << "警告: 回到初始姿态后的欧拉角零偏重采失败，保留上一份零偏" << std::endl;
            }
            std::cout << "已回到初始姿态，重新开始控制循环" << std::endl;
            std::cout << "========================================" << std::endl;
        };

        if (gamepad.consumeLTPressedEdge()) {
            std::cout << "\n[手柄] LT按下，恢复到初始姿态" << std::endl;
            enqueueEventRecord("lt_reset", "手柄LT按下，触发回到初始姿态", nullptr, nullptr, &response);
            inference.reset();
            std::fill(last_action, last_action + ACTION_DIM, 0.0f);
            std::memset(&response, 0, sizeof(response));
            for (int i = 0; i < ACTION_DIM; ++i) {
                response.q_exp[i] = calibration.init_pose[i] + calibration.offset[i];
            }
            moveToInitPose(udp, calibration.init_pose, calibration.offset);
            if (!calibrateInitEulerBias(udp, calibration.init_pose, calibration.offset, euler_bias)) {
                std::cerr << "警告: LT恢复后的欧拉角零偏重采失败，保留上一份零偏" << std::endl;
            }
            continue;
        }

        // 发送上一次的响应
        udp.sendResponse(response);

        // 接收新的请求
        if (udp.receiveRequest(request)) {
            if (gamepad.isConnected()) {
                float vx = 0.0f;
                float vy = 0.0f;
                float yaw_rate = 0.0f;
                gamepad.getCommand(vx, vy, yaw_rate);
                request.command[0] = vx;
                request.command[1] = vy;
                request.command[2] = yaw_rate;
                request.command[3] = 0.0f;
            }

            MsgRequest request_for_policy = request;
            for (int i = 0; i < ACTION_DIM; ++i) {
                request_for_policy.q[i] = request.q[i] - calibration.offset[i];
            }
            for (int i = 0; i < 3; ++i) {
                request_for_policy.eu_ang[i] = request.eu_ang[i] - euler_bias[i];
            }

            bool torque_over_limit = false;
            int torque_over_limit_joint = -1;
            float torque_over_limit_value = 0.0f;
            for (int i = 0; i < ACTION_DIM; ++i) {
                float abs_tau = std::fabs(request.tau[i]);
                if (abs_tau > kTorqueFeedbackLimitNm) {
                    torque_over_limit = true;
                    torque_over_limit_joint = i;
                    torque_over_limit_value = request.tau[i];
                    break;
                }
            }

            if (torque_over_limit) {
                enqueueEventRecord(
                    "torque_over_limit",
                    "关节力矩超限，joint[" + std::to_string(torque_over_limit_joint) +
                    "] = " + std::to_string(torque_over_limit_value) +
                    " Nm，阈值 = " + std::to_string(kTorqueFeedbackLimitNm) + " Nm",
                    &request,
                    &request_for_policy,
                    &response);
                resetToInitPose(
                    "关节力矩超限，joint[" + std::to_string(torque_over_limit_joint) +
                    "] = " + std::to_string(torque_over_limit_value) +
                    " Nm，阈值 = " + std::to_string(kTorqueFeedbackLimitNm) + " Nm");
                continue;
            }

            float action[ACTION_DIM];

            // 执行推理
            bool infer_success = inference.infer(request_for_policy, action);

            // ========== 检查输出中是否有NaN ==========
            bool has_nan = false;
            if (infer_success) {
                for (int i = 0; i < ACTION_DIM; ++i) {
                    if (std::isnan(action[i])) {
                        has_nan = true;
                        break;
                    }
                }
            }

            // ========== 推理失败、trigger!=1.0或输出有NaN时的处理 ==========
            if (infer_success && !has_nan) {
                // 推理成功且无NaN，使用推理结果
                bool motor_cmd_over_limit = false;
                int motor_cmd_over_limit_joint = -1;
                float motor_cmd_before_clip = 0.0f;
                for (int i = 0; i < ACTION_DIM; ++i) {
                    // 应用滤波: 0.8*current_action + 0.2*last_action
                    float filtered = 0.8f * action[i] + 0.2f * last_action[i];

                    // 先对相对初始姿态的偏移量按关节分别限幅，再叠加标定初始姿态
                    if (filtered < filtered_action_lower_limits[i]) filtered = filtered_action_lower_limits[i];
                    if (filtered > filtered_action_upper_limits[i]) filtered = filtered_action_upper_limits[i];
                    float motor_cmd = filtered * kActionToPositionScale + calibration.init_pose[i];

                    if (motor_cmd < kJointPosLowerLimits[i]) {
                        motor_cmd_before_clip = motor_cmd;
                        motor_cmd = kJointPosLowerLimits[i];
                        motor_cmd_over_limit = true;
                        motor_cmd_over_limit_joint = i;
                    } else if (motor_cmd > kJointPosUpperLimits[i]) {
                        motor_cmd_before_clip = motor_cmd;
                        motor_cmd = kJointPosUpperLimits[i];
                        motor_cmd_over_limit = true;
                        motor_cmd_over_limit_joint = i;
                    }

                    response.q_exp[i] = motor_cmd + calibration.offset[i];
                    last_action[i] = action[i];  // 保存原始action用于下一次滤波
                }

                if (motor_cmd_over_limit) {
                    enqueueEventRecord(
                        "motor_cmd_over_limit",
                        "最终电机目标位置超限，joint[" + std::to_string(motor_cmd_over_limit_joint) +
                        "] " + kJointNames[motor_cmd_over_limit_joint] +
                        "] = " + std::to_string(motor_cmd_before_clip) +
                        " rad，阈值 = [" + std::to_string(kJointPosLowerLimits[motor_cmd_over_limit_joint]) +
                        ", " + std::to_string(kJointPosUpperLimits[motor_cmd_over_limit_joint]) + "] rad",
                        &request,
                        &request_for_policy,
                        &response);
                    resetToInitPose(
                        "最终电机目标位置超限，joint[" + std::to_string(motor_cmd_over_limit_joint) +
                        "] " + kJointNames[motor_cmd_over_limit_joint] +
                        "] = " + std::to_string(motor_cmd_before_clip) +
                        " rad，阈值 = [" + std::to_string(kJointPosLowerLimits[motor_cmd_over_limit_joint]) +
                        ", " + std::to_string(kJointPosUpperLimits[motor_cmd_over_limit_joint]) + "] rad");
                    continue;
                }
                infer_count++;
                CsvLogRecord inference_record = buildRecordFromState(
                    &request, &request_for_policy, &response, "inference", "", "");
                last_complete_record = inference_record;
                has_last_complete_record = true;
                csv_logger.enqueue(std::move(inference_record));

                // 每0.5秒打印一次状态（250次 × 2ms = 500ms）
                if (infer_count % 250 == 0) {
                    // 获取观测向量
                    float obs[OBS_DIM];
                    inference.getLastObservation(obs);

                    std::cout << "[推理 #" << infer_count << "]" << std::endl;

                    // 打印观测值（39维）
                    std::cout << "  角速度: w_x=" << std::fixed << std::setprecision(6) << obs[0]
                              << " w_y=" << obs[1] << " w_z=" << obs[2] << std::endl;

                    std::cout << "  欧拉角: roll=" << obs[3] << " pitch=" << obs[4] << " yaw=" << obs[5] << std::endl;

                    std::cout << "  控制指令: cmd_vx=" << obs[6] << " cmd_vy=" << obs[7] << " cmd_dyaw=" << obs[8] << std::endl;

                    std::cout << "  关节位置: ";
                    for (int i = 0; i < 10; i++) {
                        std::cout << obs[9 + i] << " ";
                    }
                    std::cout << std::endl;

                    std::cout << "  关节速度: ";
                    for (int i = 0; i < 10; i++) {
                        std::cout << obs[19 + i] << " ";
                    }
                    std::cout << std::endl;

                    std::cout << "  上一步动作: ";
                    for (int i = 0; i < 10; i++) {
                        std::cout << obs[29 + i] << " ";
                    }
                    std::cout << std::endl;

                    std::cout << "  动作输出: ";
                    for (int i = 0; i < ACTION_DIM; ++i) {
                        std::cout << response.q_exp[i] << " ";
                    }
                    std::cout << std::endl;
                }
            } else {
                // 推理失败、trigger!=1.0或输出有NaN，终止推理并回到初始姿态
                if (has_nan) {
                    enqueueEventRecord("nan_output", "推理输出包含NaN，终止推理", &request, &request_for_policy, &response);
                    resetToInitPose("推理输出包含NaN，终止推理");
                } else if (!infer_success) {
                    enqueueEventRecord(
                        "infer_failed",
                        "推理失败或trigger!=1.0，trigger=" + std::to_string(request.trigger),
                        &request,
                        &request_for_policy,
                        &response);
                    resetToInitPose("推理失败或trigger!=1.0，终止推理");
                }
            }
        }

        loop_count++;
        usleep(2000);  // 2ms，500Hz
    }

    // ========== 打印退出信息 ==========
    std::cout << "\n========================================" << std::endl;
    std::cout << "程序已退出" << std::endl;
    std::cout << "总循环次数: " << loop_count << std::endl;
    std::cout << "总推理次数: " << infer_count << std::endl;
    std::cout << "========================================" << std::endl;

    enqueueEventRecord("ctrl_c", "收到Ctrl+C或终止信号，准备退出主程序", nullptr, nullptr, &response);
    csv_logger.stop();
    std::cout << "CSV复盘日志已写入: " << csv_log_path << std::endl;

    gamepad.close();

    return 0;
}
