/**
 * @file test_zero_pose.cpp
 * @brief 零位姿态测试程序
 * @author Zomnk
 * @date 2026-04-10
 *
 * @details 本程序用于将 10 个关节平滑移动到全零位并保持。
 *          功能：
 *          1. 通过 UDP 与 ODroid 建立通信
 *          2. 使用首次有效反馈的关节位置作为插值起点
 *          3. 在固定 5 秒内线性插值到全零位
 *          4. 到达后持续保持零位
 *
 * @note 使用方法:
 *       ./test_zero_pose [--ip IP] [--port PORT]
 *
 * @warning 运行前请确保机器人处于安全状态！
 */

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <iomanip>
#include <sys/time.h>

using namespace std;

#pragma pack(push, 1)
struct MsgRequest {
    float trigger;
    float command[4];
    float eu_ang[3];
    float omega[3];
    float acc[3];
    float q[10];
    float dq[10];
    float tau[10];
    float init_pos[10];
    float quat[4];
};

struct MsgResponse {
    float q_exp[10];
    float dq_exp[10];
    float tau_exp[10];
};
#pragma pack(pop)

static_assert(sizeof(MsgRequest) == 58 * sizeof(float), "MsgRequest size must be 232 bytes");
static_assert(sizeof(MsgResponse) == 30 * sizeof(float), "MsgResponse size must be 120 bytes");

volatile bool g_running = true;
constexpr float kHandshakeModeFlag = -888.0f;

void signal_handler(int sig) {
    cout << "\n收到信号 " << sig << ", 准备退出..." << endl;
    g_running = false;
}

int main(int argc, char** argv) {
    string target_ip = "192.168.5.159";
    int port = 10000;
    constexpr uint64_t kInterpolationDurationUs = 5000000ULL;  // 固定5秒插值

    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            target_ip = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = atoi(argv[++i]);
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cout << "========================================" << endl;
    cout << "  零位姿态测试程序" << endl;
    cout << "  目标: " << target_ip << ":" << port << endl;
    cout << "========================================" << endl;

    float zero_pos[10] = {0.0f};

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        cerr << "错误: 无法创建socket" << endl;
        return 1;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        cerr << "错误: 绑定端口失败" << endl;
        close(sock);
        return 1;
    }

    struct sockaddr_in remote_addr;
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(target_ip.c_str());
    remote_addr.sin_port = htons(port);

    cout << "\n正在连接ODroid..." << endl;

    MsgRequest request;
    MsgResponse response;
    memset(&response, 0, sizeof(response));
    response.dq_exp[0] = kHandshakeModeFlag;

    float startup_pos[10] = {0.0f};
    bool startup_pos_captured = false;

    for (int i = 0; i < 50 && !startup_pos_captured; i++) {
        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        int received = recvfrom(sock, (char*)&request, sizeof(request), 0,
                                (struct sockaddr*)&remote_addr, &addr_len);
        if (received == static_cast<int>(sizeof(request))) {
            for (int j = 0; j < 10; j++) {
                startup_pos[j] = request.q[j];
            }
            startup_pos_captured = true;
            cout << "已连接ODroid" << endl;
        } else if (received > 0) {
            cerr << "[警告] 收到异常长度数据包: " << received
                 << " bytes，期望 " << sizeof(request) << " bytes" << endl;
        }
        usleep(10000);
    }

    if (!startup_pos_captured) {
        cout << "警告: 未收到反馈，使用全零位作为插值起点" << endl;
        for (int i = 0; i < 10; i++) {
            startup_pos[i] = zero_pos[i];
        }
    }

    cout << "\n正在用5秒时间平滑插值到全零位..." << endl;

    memset(&response, 0, sizeof(response));

    struct timeval interp_start_tv;
    gettimeofday(&interp_start_tv, NULL);
    uint64_t interp_start_us = interp_start_tv.tv_sec * 1000000ULL + interp_start_tv.tv_usec;

    while (g_running) {
        struct timeval now_tv;
        gettimeofday(&now_tv, NULL);
        uint64_t now_us = now_tv.tv_sec * 1000000ULL + now_tv.tv_usec;
        uint64_t elapsed_us = now_us - interp_start_us;

        float alpha = static_cast<float>(elapsed_us) / static_cast<float>(kInterpolationDurationUs);
        if (alpha > 1.0f) alpha = 1.0f;

        for (int i = 0; i < 10; i++) {
            response.q_exp[i] = startup_pos[i] + alpha * (zero_pos[i] - startup_pos[i]);
            response.dq_exp[i] = 0.0f;
            response.tau_exp[i] = 0.0f;
        }

        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        int received = recvfrom(sock, (char*)&request, sizeof(request), 0,
                                (struct sockaddr*)&remote_addr, &addr_len);
        if (received > 0 && received != static_cast<int>(sizeof(request))) {
            cerr << "[警告] 收到异常长度数据包: " << received
                 << " bytes，期望 " << sizeof(request) << " bytes" << endl;
        }

        float remaining_s = static_cast<float>(
            kInterpolationDurationUs - (elapsed_us > kInterpolationDurationUs ? kInterpolationDurationUs : elapsed_us)
        ) / 1000000.0f;
        if ((static_cast<int>(remaining_s * 10) % 5) == 0) {
            cout << "\r回零中... " << fixed << setprecision(1)
                 << remaining_s << "s"
                 << " | alpha=" << setprecision(3) << alpha
                 << "      " << flush;
        }

        if (elapsed_us >= kInterpolationDurationUs) {
            break;
        }

        usleep(2000);
    }

    cout << endl;
    cout << "已到达全零位，保持位置不变..." << endl;
    cout << "按 Ctrl+C 退出" << endl;

    int loop_count = 0;
    while (g_running) {
        for (int i = 0; i < 10; i++) {
            response.q_exp[i] = zero_pos[i];
            response.dq_exp[i] = 0.0f;
            response.tau_exp[i] = 0.0f;
        }

        sendto(sock, (char*)&response, sizeof(response), 0,
               (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        socklen_t addr_len = sizeof(remote_addr);
        int received = recvfrom(sock, (char*)&request, sizeof(request), 0,
                                (struct sockaddr*)&remote_addr, &addr_len);
        if (received > 0 && received != static_cast<int>(sizeof(request))) {
            cerr << "[警告] 收到异常长度数据包: " << received
                 << " bytes，期望 " << sizeof(request) << " bytes" << endl;
        }

        loop_count++;
        if (loop_count % 250 == 0) {
            cout << "保持零位中... (循环 #" << loop_count << ")" << endl;
        }

        usleep(2000);
    }

    cout << "\n========================================" << endl;
    cout << "程序已退出" << endl;
    cout << "总循环次数: " << loop_count << endl;
    cout << "========================================" << endl;

    close(sock);
    return 0;
}
