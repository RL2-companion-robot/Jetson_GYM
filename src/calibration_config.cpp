/**
 * @file calibration_config.cpp
 * @brief 标定配置读写实现
 * @author Zomnk
 * @date 2026-04-15
 */

#include "calibration_config.h"
#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

enum class Section {
    NONE,
    INIT_POSE,
    OFFSET
};

int jointIndexFromKeyAndLeg(const std::string& key, bool in_left_leg, bool in_right_leg) {
    if (!in_left_leg && !in_right_leg) return -1;

    int base = in_left_leg ? 0 : 5;
    if (key == "yaw") return base + 0;
    if (key == "roll") return base + 1;
    if (key == "pitch") return base + 2;
    if (key == "knee") return base + 3;
    if (key == "ankle") return base + 4;
    return -1;
}

bool parseCalibrationArray(std::ifstream& f, float values[DOF_NUM], bool parse_offset) {
    std::string line;
    Section current_section = Section::NONE;
    bool in_left_leg = false;
    bool in_right_leg = false;
    bool parsed[DOF_NUM] = {false};
    int parsed_count = 0;

    while (std::getline(f, line)) {
        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue;

        std::string trimmed = line.substr(start);
        if (trimmed[0] == '#') continue;

        if (trimmed.find("init_pose:") != std::string::npos) {
            current_section = Section::INIT_POSE;
            in_left_leg = false;
            in_right_leg = false;
            continue;
        }
        if (trimmed.find("offset:") != std::string::npos) {
            current_section = Section::OFFSET;
            in_left_leg = false;
            in_right_leg = false;
            continue;
        }

        if ((parse_offset && current_section != Section::OFFSET) ||
            (!parse_offset && current_section != Section::INIT_POSE)) {
            continue;
        }

        if (trimmed.find("left_leg:") != std::string::npos) {
            in_left_leg = true;
            in_right_leg = false;
            continue;
        }
        if (trimmed.find("right_leg:") != std::string::npos) {
            in_left_leg = false;
            in_right_leg = true;
            continue;
        }

        size_t colon = trimmed.find(':');
        if (colon == std::string::npos) continue;

        std::string key = trimmed.substr(0, colon);
        size_t key_end = key.find_last_not_of(" \t");
        if (key_end == std::string::npos) continue;
        key = key.substr(0, key_end + 1);

        int joint_idx = jointIndexFromKeyAndLeg(key, in_left_leg, in_right_leg);
        if (joint_idx < 0) continue;

        std::string value_str = trimmed.substr(colon + 1);
        size_t comment_pos = value_str.find('#');
        if (comment_pos != std::string::npos) {
            value_str = value_str.substr(0, comment_pos);
        }
        size_t value_start = value_str.find_first_not_of(" \t");
        if (value_start == std::string::npos) continue;

        try {
            values[joint_idx] = std::stof(value_str.substr(value_start));
            if (!parsed[joint_idx]) {
                parsed[joint_idx] = true;
                parsed_count++;
            }
        } catch (...) {
        }
    }

    return parsed_count == DOF_NUM;
}

void writeJointBlock(std::ofstream& f, const char* block_name, const float values[DOF_NUM], int base) {
    f << "    " << block_name << ":" << std::endl;
    f << "      yaw:   " << std::fixed << std::setprecision(6) << values[base + 0] << "  # rad" << std::endl;
    f << "      roll:  " << values[base + 1] << "  # rad" << std::endl;
    f << "      pitch: " << values[base + 2] << "  # rad" << std::endl;
    f << "      knee:  " << values[base + 3] << "  # rad" << std::endl;
    f << "      ankle: " << values[base + 4] << "  # rad" << std::endl;
}

}  // namespace

void setZeroCalibrationConfig(CalibrationConfig& config) {
    std::memset(&config, 0, sizeof(config));
}

bool loadCalibrationConfig(const std::string& filename, CalibrationConfig& config) {
    setZeroCalibrationConfig(config);

    std::ifstream init_stream(filename);
    if (!init_stream.is_open()) {
        std::cerr << "无法打开标定文件: " << filename << std::endl;
        return false;
    }

    bool init_ok = parseCalibrationArray(init_stream, config.init_pose, false);
    init_stream.close();

    if (!init_ok) {
        std::cerr << "标定文件格式错误，无法完整解析 init_pose: " << filename << std::endl;
        return false;
    }

    std::ifstream offset_stream(filename);
    if (!offset_stream.is_open()) {
        return false;
    }
    bool offset_ok = parseCalibrationArray(offset_stream, config.offset, true);
    offset_stream.close();

    if (!offset_ok) {
        for (int i = 0; i < DOF_NUM; ++i) {
            config.offset[i] = 0.0f;
        }
    }

    return true;
}

bool saveCalibrationConfig(const CalibrationConfig& config, const std::string& filename) {
    std::ofstream f(filename);
    if (!f.is_open()) {
        return false;
    }

    f << "# 双足机器人标定数据" << std::endl;
    f << "# 生成时间: " << __DATE__ << " " << __TIME__ << std::endl;
    f << "# 单位: 弧度 (rad)" << std::endl;
    f << "# init_pose: 网络/策略坐标系下的初始姿态" << std::endl;
    f << "# offset: 编码器坐标系到网络坐标系的补偿量，满足 q_obs = q_raw - offset, q_cmd = q_policy + offset" << std::endl;
    f << std::endl;
    f << "robot_config:" << std::endl;
    f << "  init_pose:" << std::endl;
    writeJointBlock(f, "left_leg", config.init_pose, 0);
    writeJointBlock(f, "right_leg", config.init_pose, 5);
    f << std::endl;
    f << "  offset:" << std::endl;
    writeJointBlock(f, "left_leg", config.offset, 0);
    writeJointBlock(f, "right_leg", config.offset, 5);
    f << std::endl;
    f << "  # 说明:" << std::endl;
    f << "  # - 此文件由标定工具生成" << std::endl;
    f << "  # - 旧手动标定模式会将 offset 写为 0" << std::endl;
    f << "  # - 新仿真对齐标定模式会写入非零 offset" << std::endl;

    f.close();
    return true;
}
