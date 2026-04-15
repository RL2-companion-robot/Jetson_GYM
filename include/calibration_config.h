/**
 * @file calibration_config.h
 * @brief 标定配置读写接口
 * @author Zomnk
 * @date 2026-04-15
 *
 * @details 统一管理 init_pose + offset 两组标定数据。
 *          - init_pose: 网络/策略坐标系下的初始姿态
 *          - offset: 编码器坐标系到网络坐标系的补偿量
 */

#ifndef CALIBRATION_CONFIG_H
#define CALIBRATION_CONFIG_H

#include "types.h"
#include <string>

struct CalibrationConfig {
    float init_pose[DOF_NUM];
    float offset[DOF_NUM];
};

void setZeroCalibrationConfig(CalibrationConfig& config);
bool loadCalibrationConfig(const std::string& filename, CalibrationConfig& config);
bool saveCalibrationConfig(const CalibrationConfig& config, const std::string& filename);

#endif  // CALIBRATION_CONFIG_H
