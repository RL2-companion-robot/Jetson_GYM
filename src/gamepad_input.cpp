/**
 * @file gamepad_input.cpp
 * @brief 手柄输入模块实现文件
 * @author Zomnk
 * @date 2026-03-25
 */

#include "gamepad_input.h"

#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <iostream>

GamepadInput::GamepadInput()
    : fd_(-1)
    , connected_(false)
    , vx_(0.0f)
    , vy_(0.0f)
    , yaw_rate_(0.0f)
    , lt_pressed_(false)
    , lt_pressed_edge_(false) {
    std::fill(axis_values_, axis_values_ + AXIS_COUNT, 0);
    std::fill(button_values_, button_values_ + BUTTON_COUNT, 0);
}

GamepadInput::~GamepadInput() {
    close();
}

bool GamepadInput::init(const std::string& device) {
    close();

    device_path_ = device;
    fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        connected_ = false;
        return false;
    }

    connected_ = true;
    vx_ = 0.0f;
    vy_ = 0.0f;
    yaw_rate_ = 0.0f;
    lt_pressed_ = false;
    lt_pressed_edge_ = false;
    std::fill(axis_values_, axis_values_ + AXIS_COUNT, 0);
    std::fill(button_values_, button_values_ + BUTTON_COUNT, 0);
    return true;
}

void GamepadInput::update() {
    if (!connected_ || fd_ < 0) {
        return;
    }

    js_event event;
    while (true) {
        ssize_t bytes = read(fd_, &event, sizeof(event));
        if (bytes == sizeof(event)) {
            unsigned char event_type = event.type & ~JS_EVENT_INIT;

            if (event_type == JS_EVENT_AXIS) {
                if (event.number < AXIS_COUNT) {
                    axis_values_[event.number] = event.value;
                    updateCommandFromAxes();
                }
            } else if (event_type == JS_EVENT_BUTTON) {
                if (event.number < BUTTON_COUNT) {
                    bool previous = button_values_[event.number] != 0;
                    button_values_[event.number] = event.value;
                    bool current = event.value != 0;

                    if (event.number == LT_BUTTON_ID) {
                        if (!previous && current) {
                            lt_pressed_edge_ = true;
                        }
                        lt_pressed_ = current;
                    }
                }
            }
        } else if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            if (errno == EINTR) {
                continue;
            }

            std::cerr << "[Gamepad] 读取失败: " << std::strerror(errno) << std::endl;
            close();
            break;
        } else {
            close();
            break;
        }
    }
}

bool GamepadInput::isConnected() const {
    return connected_;
}

void GamepadInput::getCommand(float& vx, float& vy, float& yaw_rate) const {
    vx = vx_;
    vy = vy_;
    yaw_rate = yaw_rate_;
}

bool GamepadInput::consumeLTPressedEdge() {
    bool pressed = lt_pressed_edge_;
    lt_pressed_edge_ = false;
    return pressed;
}

void GamepadInput::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    connected_ = false;
    vx_ = 0.0f;
    vy_ = 0.0f;
    yaw_rate_ = 0.0f;
    lt_pressed_ = false;
    lt_pressed_edge_ = false;
    std::fill(axis_values_, axis_values_ + AXIS_COUNT, 0);
    std::fill(button_values_, button_values_ + BUTTON_COUNT, 0);
}

float GamepadInput::normalizeAxis(int raw) const {
    float normalized = static_cast<float>(raw) / AXIS_MAX_RAW;
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;
    return normalized;
}

float GamepadInput::applyDeadzone(float value) const {
    return std::fabs(value) < DEADZONE ? 0.0f : value;
}

void GamepadInput::updateCommandFromAxes() {
    float left_x = 0.0f;
    float left_y = 0.0f;
    float right_x = 0.0f;

    if (LEFT_X_AXIS_ID < AXIS_COUNT) {
        left_x = applyDeadzone(normalizeAxis(axis_values_[LEFT_X_AXIS_ID]));
    }
    if (LEFT_Y_AXIS_ID < AXIS_COUNT) {
        left_y = applyDeadzone(normalizeAxis(axis_values_[LEFT_Y_AXIS_ID]));
    }
    if (RIGHT_X_AXIS_ID < AXIS_COUNT) {
        right_x = applyDeadzone(normalizeAxis(axis_values_[RIGHT_X_AXIS_ID]));
    }

    vx_ = -left_y * MAX_VX;
    vy_ = left_x * MAX_VY;
    yaw_rate_ = right_x * MAX_YAW_RATE;
}
