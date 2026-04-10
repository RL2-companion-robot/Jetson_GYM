/**
 * @file test_capture_init_pose_observation.cpp
 * @brief init_pose静站阶段观测采集程序
 *
 * @details 本程序用于在Jetson上实时接收ODroid状态数据，
 *          先用5秒时间平滑移动到robot.yaml定义的init_pose，
 *          然后持续保持init_pose，同时导出策略输入对应的39维观测。
 *
 *          注意：
 *          1. 本程序不运行策略控制，不使用推理输出控制机器人
 *          2. 观测中的command固定为0，用于对齐静站阶段仿真观测
 *          3. 观测中的last_action固定为0
 *
 * @note 使用方法:
 *       ./test_capture_init_pose_observation [--ip IP] [--port PORT] [--config FILE]
 */

#include "types.h"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace {
constexpr uint64_t kInterpolationDurationUs = 5000000ULL;
constexpr float OMEGA_SCALE = 0.25f;
constexpr float EU_ANG_SCALE = 1.0f;
constexpr float POS_SCALE = 1.0f;
constexpr float VEL_SCALE = 0.05f;
constexpr float LIN_VEL_SCALE = 2.0f;
constexpr float ANG_VEL_SCALE = 0.25f;
constexpr float SMOOTH = 0.03f;
constexpr float DEAD_ZONE = 0.01f;
constexpr int kFlushEveryRows = 100;
}

volatile bool g_running = true;

void signal_handler(int sig) {
    std::cout << "\n收到信号 " << sig << ", 准备退出观测采集..." << std::endl;
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

std::string buildCsvPath() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm;
    localtime_r(&now_time, &local_tm);

    std::ostringstream oss;
    oss << "data/init_pose_obs_capture_" << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
    return oss.str();
}

std::string formatTimestamp(const std::chrono::system_clock::time_point& timestamp) {
    const auto time_t_value = std::chrono::system_clock::to_time_t(timestamp);
    std::tm local_tm;
    localtime_r(&time_t_value, &local_tm);
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
        timestamp.time_since_epoch()) % std::chrono::seconds(1);

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
        << "." << std::setw(6) << std::setfill('0') << micros.count();
    return oss.str();
}

bool load_init_pose(const std::string& filename, float init_pos[DOF_NUM]) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        std::cerr << "错误: 无法打开配置文件 " << filename << std::endl;
        return false;
    }

    bool in_left_leg = false;
    bool in_right_leg = false;
    int count = 0;
    std::string line;

    while (std::getline(f, line)) {
        size_t first_char = line.find_first_not_of(" \t");
        if (first_char != std::string::npos && line[first_char] == '#') {
            continue;
        }

        if (line.find("left_leg:") != std::string::npos) {
            in_left_leg = true;
            in_right_leg = false;
            continue;
        }
        if (line.find("right_leg:") != std::string::npos) {
            in_left_leg = false;
            in_right_leg = true;
            continue;
        }

        size_t pos = line.find(':');
        if (pos == std::string::npos) {
            continue;
        }

        std::string val = line.substr(pos + 1);
        size_t comment = val.find('#');
        if (comment != std::string::npos) {
            val = val.substr(0, comment);
        }

        size_t start = val.find_first_not_of(" \t");
        if (start == std::string::npos) {
            continue;
        }

        try {
            float value = std::stof(val.substr(start));
            int offset = in_left_leg ? 0 : 5;

            if (line.find("yaw") != std::string::npos) init_pos[offset + 0] = value;
            else if (line.find("roll") != std::string::npos) init_pos[offset + 1] = value;
            else if (line.find("pitch") != std::string::npos) init_pos[offset + 2] = value;
            else if (line.find("knee") != std::string::npos) init_pos[offset + 3] = value;
            else if (line.find("ankle") != std::string::npos) init_pos[offset + 4] = value;

            count++;
        } catch (...) {
        }
    }

    return count == 10;
}

float applyDeadzone(float value, float deadzone) {
    return std::fabs(value) < deadzone ? 0.0f : value;
}

void buildObservationForInitPose(const MsgRequest& request,
                                 const float init_pos[DOF_NUM],
                                 float& cmd_x,
                                 float& cmd_y,
                                 float& cmd_rate,
                                 float obs[OBS_DIM]) {
    int idx = 0;

    obs[idx++] = request.omega[0] * OMEGA_SCALE;
    obs[idx++] = request.omega[1] * OMEGA_SCALE;
    obs[idx++] = request.omega[2] * OMEGA_SCALE;

    obs[idx++] = request.eu_ang[0] * EU_ANG_SCALE;
    obs[idx++] = request.eu_ang[1] * EU_ANG_SCALE;
    obs[idx++] = request.eu_ang[2] * EU_ANG_SCALE;

    cmd_x = cmd_x * (1.0f - SMOOTH) + applyDeadzone(0.0f, DEAD_ZONE) * SMOOTH;
    cmd_y = cmd_y * (1.0f - SMOOTH) + applyDeadzone(0.0f, DEAD_ZONE) * SMOOTH;
    cmd_rate = cmd_rate * (1.0f - SMOOTH) + applyDeadzone(0.0f, DEAD_ZONE) * SMOOTH;
    obs[idx++] = cmd_x * LIN_VEL_SCALE;
    obs[idx++] = cmd_y * LIN_VEL_SCALE;
    obs[idx++] = cmd_rate * ANG_VEL_SCALE;

    for (int i = 0; i < DOF_NUM; ++i) {
        obs[idx++] = (request.q[i] - init_pos[i]) * POS_SCALE;
    }

    for (int i = 0; i < DOF_NUM; ++i) {
        obs[idx++] = request.dq[i] * VEL_SCALE;
    }

    for (int i = 0; i < ACTION_DIM; ++i) {
        obs[idx++] = 0.0f;
    }
}

void writeCsvHeader(std::ofstream& csv) {
    csv << "timestamp_local";
    for (int i = 0; i < OBS_DIM; ++i) csv << ",obs_" << i;
    for (int i = 0; i < DOF_NUM; ++i) csv << ",q_" << i;
    for (int i = 0; i < DOF_NUM; ++i) csv << ",dq_" << i;
    csv << ",omega_x,omega_y,omega_z";
    csv << ",acc_x,acc_y,acc_z";
    csv << ",roll,pitch,yaw";
    csv << ",quat_w,quat_x,quat_y,quat_z";
    csv << "\n";
}

void writeCsvRow(std::ofstream& csv,
                 const std::chrono::system_clock::time_point& timestamp,
                 const float obs[OBS_DIM],
                 const MsgRequest& request) {
    csv << formatTimestamp(timestamp);
    for (int i = 0; i < OBS_DIM; ++i) csv << "," << obs[i];
    for (int i = 0; i < DOF_NUM; ++i) csv << "," << request.q[i];
    for (int i = 0; i < DOF_NUM; ++i) csv << "," << request.dq[i];
    for (int i = 0; i < 3; ++i) csv << "," << request.omega[i];
    for (int i = 0; i < 3; ++i) csv << "," << request.acc[i];
    for (int i = 0; i < 3; ++i) csv << "," << request.eu_ang[i];
    for (int i = 0; i < 4; ++i) csv << "," << request.quat[i];
    csv << "\n";
}

int main(int argc, char** argv) {
    std::string target_ip = "192.168.5.159";
    int port = 10000;
    std::string config_file = "../robot.yaml";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    float init_pos[DOF_NUM] = {0.0f};
    if (!load_init_pose(config_file, init_pos)) {
        return 1;
    }

    if (!ensureDirectoryExists("data")) {
        return 1;
    }

    const std::string csv_path = buildCsvPath();
    std::ofstream csv(csv_path);
    if (!csv.is_open()) {
        std::cerr << "错误: 无法创建CSV文件 " << csv_path << std::endl;
        return 1;
    }
    writeCsvHeader(csv);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "错误: 无法创建socket" << std::endl;
        return 1;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in local_addr;
    std::memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "错误: 绑定端口失败" << std::endl;
        close(sock);
        return 1;
    }

    struct sockaddr_in remote_addr;
    std::memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(target_ip.c_str());
    remote_addr.sin_port = htons(port);

    std::cout << "========================================" << std::endl;
    std::cout << "  init_pose静站观测采集程序" << std::endl;
    std::cout << "  目标: " << target_ip << ":" << port << std::endl;
    std::cout << "  CSV: " << csv_path << std::endl;
    std::cout << "========================================" << std::endl;

    MsgRequest request;
    MsgResponse response;
    std::memset(&response, 0, sizeof(response));

    float startup_pos[DOF_NUM] = {0.0f};
    bool startup_pos_captured = false;

    std::cout << "正在连接ODroid..." << std::endl;
    for (int i = 0; i < 50 && !startup_pos_captured && g_running; ++i) {
        sendto(sock, reinterpret_cast<char*>(&response), sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        int received = recvfrom(sock, reinterpret_cast<char*>(&request), sizeof(request), 0,
                                (struct sockaddr*)&remote_addr, &addr_len);
        if (received == static_cast<int>(sizeof(request))) {
            for (int j = 0; j < DOF_NUM; ++j) {
                startup_pos[j] = request.q[j];
            }
            startup_pos_captured = true;
            std::cout << "已连接ODroid" << std::endl;
        } else if (received > 0) {
            std::cerr << "[警告] 收到异常长度数据包: " << received
                      << " bytes，期望 " << sizeof(request) << " bytes" << std::endl;
        }
        usleep(10000);
    }

    if (!startup_pos_captured) {
        std::cout << "警告: 未收到反馈，使用 init_pose 作为插值起点" << std::endl;
        for (int i = 0; i < DOF_NUM; ++i) {
            startup_pos[i] = init_pos[i];
        }
    }

    std::cout << "用5秒时间平滑插值到初始姿态..." << std::endl;
    auto interp_start = std::chrono::steady_clock::now();
    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        uint64_t elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - interp_start).count();
        float alpha = static_cast<float>(elapsed_us) / static_cast<float>(kInterpolationDurationUs);
        if (alpha > 1.0f) alpha = 1.0f;

        for (int i = 0; i < DOF_NUM; ++i) {
            response.q_exp[i] = startup_pos[i] + alpha * (init_pos[i] - startup_pos[i]);
            response.dq_exp[i] = 0.0f;
            response.tau_exp[i] = 0.0f;
        }

        sendto(sock, reinterpret_cast<char*>(&response), sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        int received = recvfrom(sock, reinterpret_cast<char*>(&request), sizeof(request), 0,
                                (struct sockaddr*)&remote_addr, &addr_len);
        if (received > 0 && received != static_cast<int>(sizeof(request))) {
            std::cerr << "[警告] 收到异常长度数据包: " << received
                      << " bytes，期望 " << sizeof(request) << " bytes" << std::endl;
        }

        float remaining_s = static_cast<float>(
            kInterpolationDurationUs - (elapsed_us > kInterpolationDurationUs ? kInterpolationDurationUs : elapsed_us)
        ) / 1000000.0f;
        if ((static_cast<int>(remaining_s * 10) % 5) == 0) {
            std::cout << "\r初始化中... " << std::fixed << std::setprecision(1)
                      << remaining_s << "s"
                      << " | alpha=" << std::setprecision(3) << alpha
                      << "      " << std::flush;
        }

        if (elapsed_us >= kInterpolationDurationUs) {
            break;
        }

        usleep(2000);
    }
    std::cout << std::endl;

    for (int i = 0; i < DOF_NUM; ++i) {
        response.q_exp[i] = init_pos[i];
        response.dq_exp[i] = 0.0f;
        response.tau_exp[i] = 0.0f;
    }
    sendto(sock, reinterpret_cast<char*>(&response), sizeof(response), 0,
           (struct sockaddr*)&remote_addr, sizeof(remote_addr));

    float cmd_x = 0.0f;
    float cmd_y = 0.0f;
    float cmd_rate = 0.0f;
    uint64_t sample_count = 0;

    std::cout << "开始采集静站观测，按 Ctrl+C 退出" << std::endl;

    while (g_running) {
        sendto(sock, reinterpret_cast<char*>(&response), sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        int received = recvfrom(sock, reinterpret_cast<char*>(&request), sizeof(request), 0,
                                (struct sockaddr*)&remote_addr, &addr_len);

        if (received == static_cast<int>(sizeof(request))) {
            float obs[OBS_DIM] = {0.0f};
            buildObservationForInitPose(request, init_pos, cmd_x, cmd_y, cmd_rate, obs);

            writeCsvRow(csv, std::chrono::system_clock::now(), obs, request);
            sample_count++;

            if (sample_count % kFlushEveryRows == 0) {
                csv.flush();
                std::cout << "[采集] 已记录 " << sample_count << " 帧观测" << std::endl;
            }
        } else if (received > 0) {
            std::cerr << "[警告] 收到异常长度数据包: " << received
                      << " bytes，期望 " << sizeof(request) << " bytes" << std::endl;
        }

        usleep(2000);
    }

    csv.flush();
    csv.close();
    close(sock);

    std::cout << "========================================" << std::endl;
    std::cout << "观测采集结束" << std::endl;
    std::cout << "总采样帧数: " << sample_count << std::endl;
    std::cout << "CSV文件: " << csv_path << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
