/**
 * @file gamepad_calibration.cpp
 * @brief 基于 joystick 接口的手柄调试工具
 * @details 读取 /dev/input/js0，显示轴值、按钮值以及映射后的速度命令
 *
 * 运行: ./gamepad_calibration [device_path]
 * 例如: ./gamepad_calibration /dev/input/js0
 */

#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <linux/joystick.h>
#include <sys/stat.h>

namespace {
constexpr int AXIS_COUNT = 8;
constexpr int BUTTON_COUNT = 16;
constexpr int LEFT_X_AXIS_ID = 0;
constexpr int LEFT_Y_AXIS_ID = 1;
constexpr int RIGHT_X_AXIS_ID = 2;
constexpr int LT_BUTTON_ID = 8;
constexpr float AXIS_MAX_RAW = 32767.0f;
constexpr float DEADZONE = 0.10f;
constexpr float MAX_VX = 0.2f;
constexpr float MAX_VY = 0.2f;
constexpr float MAX_YAW_RATE = 0.4f;

float normalizeAxis(int raw) {
    float v = static_cast<float>(raw) / AXIS_MAX_RAW;
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
}

float applyDeadzone(float value) {
    return std::fabs(value) < DEADZONE ? 0.0f : value;
}
}

int main(int argc, char* argv[]) {
    const char* device = "/dev/input/js0";
    if (argc > 1) {
        device = argv[1];
    }

    std::cout << "========================================" << std::endl;
    std::cout << "  手柄调试工具 (joystick/js0)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "\n尝试打开设备: " << device << std::endl;

    struct stat st;
    if (stat(device, &st) < 0) {
        std::cerr << "设备文件不存在: " << device << std::endl;
        return 1;
    }

    int fd = open(device, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "无法打开设备: " << device << std::endl;
        std::cerr << "错误: " << std::strerror(errno) << std::endl;
        return 1;
    }

    int axis_values[AXIS_COUNT] = {0};
    int button_values[BUTTON_COUNT] = {0};

    std::cout << "设备已打开成功！" << std::endl;
    std::cout << "\n映射方案A：" << std::endl;
    std::cout << "  左摇杆前后 -> vx (±0.2 m/s)" << std::endl;
    std::cout << "  左摇杆左右 -> vy (±0.2 m/s)" << std::endl;
    std::cout << "  右摇杆左右 -> yaw_rate (±0.4 rad/s)" << std::endl;
    std::cout << "  LT按钮号 -> " << LT_BUTTON_ID << std::endl;
    std::cout << "\n按 Ctrl+C 退出\n" << std::endl;

    int event_count = 0;
    while (true) {
        js_event event;
        ssize_t bytes = read(fd, &event, sizeof(event));

        if (bytes == sizeof(event)) {
            unsigned char event_type = event.type & ~JS_EVENT_INIT;
            event_count++;

            if (event_type == JS_EVENT_AXIS && event.number < AXIS_COUNT) {
                axis_values[event.number] = event.value;

                float left_x = applyDeadzone(normalizeAxis(axis_values[LEFT_X_AXIS_ID]));
                float left_y = applyDeadzone(normalizeAxis(axis_values[LEFT_Y_AXIS_ID]));
                float right_x = applyDeadzone(normalizeAxis(axis_values[RIGHT_X_AXIS_ID]));

                float vx = -left_y * MAX_VX;
                float vy = left_x * MAX_VY;
                float yaw_rate = right_x * MAX_YAW_RATE;

                std::cout << "[轴事件 #" << event_count << "] 轴号: " << std::setw(2) << static_cast<int>(event.number)
                          << " | 值: " << std::setw(6) << event.value
                          << " | 归一化: " << std::fixed << std::setprecision(3)
                          << normalizeAxis(event.value)
                          << " | vx=" << vx
                          << " vy=" << vy
                          << " yaw=" << yaw_rate << std::endl;
            } else if (event_type == JS_EVENT_BUTTON && event.number < BUTTON_COUNT) {
                button_values[event.number] = event.value;
                std::cout << "[按钮事件 #" << event_count << "] 按钮号: " << std::setw(2) << static_cast<int>(event.number)
                          << " | 状态: " << (event.value ? "按下" : "释放");
                if (event.number == LT_BUTTON_ID && event.value) {
                    std::cout << " | LT触发";
                }
                std::cout << std::endl;
            }
        } else if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(10000);
                continue;
            }
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "读取错误: " << std::strerror(errno) << std::endl;
            break;
        } else if (bytes == 0) {
            std::cerr << "设备已关闭" << std::endl;
            break;
        }
    }

    close(fd);
    return 0;
}
