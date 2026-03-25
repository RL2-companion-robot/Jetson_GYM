/**
 * @file gamepad_input.h
 * @brief 手柄输入模块头文件
 * @author Zomnk
 * @date 2026-03-25
 *
 * @details 封装 Linux joystick API，读取 /dev/input/js0 手柄输入，
 *          并转换为标准化速度命令与按钮边沿事件。
 */

#ifndef GAMEPAD_INPUT_H
#define GAMEPAD_INPUT_H

#include <string>

class GamepadInput {
public:
    GamepadInput();
    ~GamepadInput();

    bool init(const std::string& device = "/dev/input/js0");
    void update();
    bool isConnected() const;
    void getCommand(float& vx, float& vy, float& yaw_rate) const;
    bool consumeLTPressedEdge();
    void close();

private:
    float normalizeAxis(int raw) const;
    float applyDeadzone(float value) const;
    void updateCommandFromAxes();

    static constexpr int AXIS_COUNT = 8;
    static constexpr int BUTTON_COUNT = 16;

    // 已确认的 joystick 按钮编号
    static constexpr int LT_BUTTON_ID = 8;

    // 默认轴编号，若手柄实际编号不同可在此调整
    static constexpr int LEFT_X_AXIS_ID = 0;
    static constexpr int LEFT_Y_AXIS_ID = 1;
    static constexpr int RIGHT_X_AXIS_ID = 2;

    static constexpr float MAX_VX = 0.2f;
    static constexpr float MAX_VY = 0.2f;
    static constexpr float MAX_YAW_RATE = 0.4f;
    static constexpr float AXIS_MAX_RAW = 32767.0f;
    static constexpr float DEADZONE = 0.10f;

    int fd_;
    bool connected_;
    std::string device_path_;

    float vx_;
    float vy_;
    float yaw_rate_;

    int axis_values_[AXIS_COUNT];
    int button_values_[BUTTON_COUNT];

    bool lt_pressed_;
    bool lt_pressed_edge_;
};

#endif // GAMEPAD_INPUT_H
