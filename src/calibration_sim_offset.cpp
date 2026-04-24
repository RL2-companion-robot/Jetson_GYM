/**
 * @file calibration_sim_offset.cpp
 * @brief 仿真初始姿态对齐标定工具
 * @author Zomnk
 * @date 2026-04-15
 *
 * @details 将机器人平滑移动到代码中写死的仿真 init_pose，
 *          保持稳定后读取编码器平均值，计算 offset = encoder_raw - sim_init_pose，
 *          并以统一 YAML 接口保存 init_pose + offset。
 */

#include "calibration_config.h"
#include "types.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <sys/time.h>
#include <cmath>
#include <array>
#include <limits>

using namespace std;

namespace {

volatile bool g_running = true;

constexpr uint64_t kMoveDurationUs = 5000000ULL;
constexpr uint64_t kSettleDurationUs = 2000000ULL;
constexpr uint64_t kSampleWindowUs = 1000000ULL;
constexpr float kTorqueFeedbackLimitNm = 4.0f;
constexpr float kHandshakeModeFlag = -888.0f;

constexpr float kSimInitPose[DOF_NUM] = {
    0.0f, 0.0f, 0.18f, 1.11f, 0.92f,
    0.0f, 0.0f, -0.18f, -1.11f, -0.92f
};

constexpr std::array<const char*, DOF_NUM> kJointNames = {{
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

constexpr std::array<float, DOF_NUM> kJointPosLowerLimits = {{
    -0.20f, -0.09f, -1.57f, -1.57f, -1.35f,
    -0.76f, -0.09f, -1.57f, -1.57f, -1.35f
}};

constexpr std::array<float, DOF_NUM> kJointPosUpperLimits = {{
     0.76f,  0.56f,  1.57f,  1.57f,  1.57f,
     0.20f,  0.56f,  1.57f,  1.57f,  1.57f
}};

void signal_handler(int sig) {
    cout << "\n收到信号 " << sig << ", 准备退出仿真对齐标定..." << endl;
    g_running = false;
}

uint64_t nowUs() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000000ULL + tv.tv_usec;
}

bool checkTargetWithinJointLimits(const float target[DOF_NUM], std::string& reason) {
    for (int i = 0; i < DOF_NUM; ++i) {
        if (target[i] < kJointPosLowerLimits[i] || target[i] > kJointPosUpperLimits[i]) {
            reason = std::string("目标姿态超出关节限位，")
                + kJointNames[i] + " = " + std::to_string(target[i])
                + " rad, limit=[" + std::to_string(kJointPosLowerLimits[i])
                + ", " + std::to_string(kJointPosUpperLimits[i]) + "]";
            return false;
        }
    }
    return true;
}

bool checkTorqueOverLimit(const MsgRequest& request, std::string& reason) {
    for (int i = 0; i < DOF_NUM; ++i) {
        if (std::fabs(request.tau[i]) > kTorqueFeedbackLimitNm) {
            reason = std::string("关节力矩超限，")
                + kJointNames[i] + " = " + std::to_string(request.tau[i])
                + " Nm, threshold=" + std::to_string(kTorqueFeedbackLimitNm) + " Nm";
            return true;
        }
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    string ip = "192.168.5.159";
    int port = 10000;
    string output = "../robot_sim_offset_calibration.yaml";

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--output" && i + 1 < argc) {
            output = argv[++i];
        } else if (arg == "--ip" && i + 1 < argc) {
            ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = atoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            cout << "使用方法:\n";
            cout << "  " << argv[0] << " [选项]\n\n";
            cout << "选项:\n";
            cout << "  --output <FILE>   指定输出文件 (默认: ../robot_sim_offset_calibration.yaml)\n";
            cout << "  --ip <IP>         ODroid IP地址\n";
            cout << "  --port <PORT>     UDP端口 (默认: 10000)\n";
            return 0;
        }
    }

    cout << "========================================" << endl;
    cout << "   仿真初始姿态对齐标定工具" << endl;
    cout << "========================================" << endl;
    cout << "通信配置: " << ip << ":" << port << endl;
    cout << "输出文件: " << output << endl;
    cout << "仿真 init_pose: [";
    for (int i = 0; i < DOF_NUM; ++i) {
        cout << fixed << setprecision(3) << kSimInitPose[i];
        if (i + 1 < DOF_NUM) cout << ", ";
    }
    cout << "]" << endl;
    cout << "========================================" << endl;

    std::string limit_reason;
    if (!checkTargetWithinJointLimits(kSimInitPose, limit_reason)) {
        cerr << "错误: " << limit_reason << endl;
        return 1;
    }

    cout << "安全提示:" << endl;
    cout << "  1. 请确认机器人处于保护状态（建议吊架/支撑）" << endl;
    cout << "  2. 请确认周围无人接触机器人" << endl;
    cout << "  3. 程序将用 5 秒时间插值到仿真 init_pose" << endl;
    cout << "输入大写 YES 后继续，其他输入取消: " << flush;
    string confirmation;
    std::getline(cin, confirmation);
    if (confirmation != "YES") {
        cout << "已取消标定。" << endl;
        return 0;
    }

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        cerr << "创建socket失败!" << endl;
        return 1;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in local_addr, remote_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(port);

    if (bind(sock_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        cerr << "Bind端口失败! 请检查端口 " << port << " 是否被占用" << endl;
        close(sock_fd);
        return 1;
    }

    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    remote_addr.sin_port = htons(port);

    cout << "\n等待与ODroid建立连接..." << endl;

    socklen_t addr_len = sizeof(remote_addr);
    MsgRequest request;
    MsgResponse response;
    memset(&response, 0, sizeof(response));
    response.dq_exp[0] = kHandshakeModeFlag;

    bool connected = false;
    float startup_pos[DOF_NUM] = {0.0f};
    while (!connected && g_running) {
        sendto(sock_fd, &response, sizeof(response), 0, (struct sockaddr*)&remote_addr, addr_len);
        int received = recvfrom(sock_fd, &request, sizeof(request), 0, (struct sockaddr*)&remote_addr, &addr_len);
        if (received == static_cast<int>(sizeof(MsgRequest))) {
            std::string torque_reason;
            if (checkTorqueOverLimit(request, torque_reason)) {
                cerr << "错误: 建立连接阶段检测到 " << torque_reason << endl;
                close(sock_fd);
                return 1;
            }
            for (int i = 0; i < DOF_NUM; ++i) {
                startup_pos[i] = request.q[i];
                response.q_exp[i] = startup_pos[i];
                response.dq_exp[i] = 0.0f;
                response.tau_exp[i] = 0.0f;
            }
            connected = true;
            // 收到首帧反馈后，先回发当前位姿保持，避免连接阶段给出全零指令
            sendto(sock_fd, &response, sizeof(response), 0, (struct sockaddr*)&remote_addr, addr_len);
            cout << "✓ 已连接到ODroid，当前位姿保持完成，开始移动到仿真 init_pose..." << endl;
        } else if (received > 0) {
            cout << "[警告] 收到异常长度数据包: " << received
                 << " bytes，期望 " << sizeof(MsgRequest) << " bytes" << endl;
        }
        usleep(10000);
    }

    if (!g_running) {
        close(sock_fd);
        return 0;
    }

    uint64_t move_start_us = nowUs();
    while (g_running) {
        uint64_t elapsed_us = nowUs() - move_start_us;
        float alpha = static_cast<float>(elapsed_us) / static_cast<float>(kMoveDurationUs);
        if (alpha > 1.0f) alpha = 1.0f;

        for (int i = 0; i < DOF_NUM; ++i) {
            response.q_exp[i] = startup_pos[i] + alpha * (kSimInitPose[i] - startup_pos[i]);
            response.dq_exp[i] = 0.0f;
            response.tau_exp[i] = 0.0f;
        }

        std::string move_limit_reason;
        if (!checkTargetWithinJointLimits(response.q_exp, move_limit_reason)) {
            cerr << "错误: 插值过程目标超限: " << move_limit_reason << endl;
            close(sock_fd);
            return 1;
        }

        sendto(sock_fd, &response, sizeof(response), 0, (struct sockaddr*)&remote_addr, addr_len);
        int received = recvfrom(sock_fd, &request, sizeof(request), 0, (struct sockaddr*)&remote_addr, &addr_len);
        if (received == static_cast<int>(sizeof(MsgRequest))) {
            std::string torque_reason;
            if (checkTorqueOverLimit(request, torque_reason)) {
                cerr << "错误: 运动阶段检测到 " << torque_reason << endl;
                close(sock_fd);
                return 1;
            }
        } else if (received > 0) {
            cout << "[警告] 收到异常长度数据包: " << received
                 << " bytes，期望 " << sizeof(MsgRequest) << " bytes" << endl;
        }

        if (elapsed_us >= kMoveDurationUs) {
            break;
        }

        usleep(2000);
    }

    cout << "已到达仿真 init_pose，开始稳定保持并采样编码器..." << endl;

    float encoder_sum[DOF_NUM] = {0.0f};
    int sample_count = 0;
    uint64_t settle_start_us = nowUs();
    while (g_running) {
        uint64_t elapsed_us = nowUs() - settle_start_us;
        bool in_sampling_window = elapsed_us >= (kSettleDurationUs - kSampleWindowUs);

        for (int i = 0; i < DOF_NUM; ++i) {
            response.q_exp[i] = kSimInitPose[i];
            response.dq_exp[i] = 0.0f;
            response.tau_exp[i] = 0.0f;
        }

        std::string settle_limit_reason;
        if (!checkTargetWithinJointLimits(response.q_exp, settle_limit_reason)) {
            cerr << "错误: 保持阶段目标超限: " << settle_limit_reason << endl;
            close(sock_fd);
            return 1;
        }

        sendto(sock_fd, &response, sizeof(response), 0, (struct sockaddr*)&remote_addr, addr_len);
        int received = recvfrom(sock_fd, &request, sizeof(request), 0, (struct sockaddr*)&remote_addr, &addr_len);
        if (received == static_cast<int>(sizeof(MsgRequest))) {
            std::string torque_reason;
            if (checkTorqueOverLimit(request, torque_reason)) {
                cerr << "错误: 保持阶段检测到 " << torque_reason << endl;
                close(sock_fd);
                return 1;
            }
            if (in_sampling_window) {
                for (int i = 0; i < DOF_NUM; ++i) {
                    encoder_sum[i] += request.q[i];
                }
                sample_count++;
            }
        } else if (received > 0 && received != static_cast<int>(sizeof(MsgRequest))) {
            cout << "[警告] 收到异常长度数据包: " << received
                 << " bytes，期望 " << sizeof(MsgRequest) << " bytes" << endl;
        }

        if (elapsed_us >= kSettleDurationUs) {
            break;
        }

        usleep(2000);
    }

    if (!g_running) {
        close(sock_fd);
        return 0;
    }

    if (sample_count <= 0) {
        cerr << "错误: 未采集到有效编码器样本，标定失败" << endl;
        close(sock_fd);
        return 1;
    }

    CalibrationConfig config;
    setZeroCalibrationConfig(config);

    cout << "\n========================================" << endl;
    cout << "仿真对齐标定结果" << endl;
    cout << "========================================" << endl;
    for (int i = 0; i < DOF_NUM; ++i) {
        float encoder_mean = encoder_sum[i] / static_cast<float>(sample_count);
        config.init_pose[i] = kSimInitPose[i];
        config.offset[i] = encoder_mean - kSimInitPose[i];
        cout << "[" << i << "] sim=" << fixed << setprecision(4) << kSimInitPose[i]
             << " raw_mean=" << encoder_mean
             << " offset=" << config.offset[i] << endl;
    }
    cout << "样本数: " << sample_count << endl;
    cout << "========================================" << endl;

    cout << "\n保存到文件: " << output << " ... ";
    if (saveCalibrationConfig(config, output)) {
        cout << "✓ 成功!" << endl;
    } else {
        cout << "✗ 失败!" << endl;
        close(sock_fd);
        return 1;
    }

    close(sock_fd);
    return 0;
}
