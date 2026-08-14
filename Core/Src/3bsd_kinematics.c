//
// Created by hp_cc on 2026/7/13.
//
#include "3bsd_kinematics.h"
#include <math.h>

// 辅助限幅函数
static double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

bool solveInverseKinematics(double delta_N, double delta_Ny, double theta,
                            double* omega1, double* omega2, double* omega3)
{
    // 1. 物理边界安全保护：限制最大偏转角 delta_N_max = 4 * theta
    double max_delta_N = 105.0;
    double checked_delta_N = clamp(delta_N, 0.0, max_delta_N);

    // 将所有输入角度转换为弧度
    double delta_N_rad  = checked_delta_N * M_PI / 180.0;
    double delta_Ny_rad = delta_Ny * M_PI / 180.0;
    double theta_rad    = theta * M_PI / 180.0;

    // 2. 特殊奇异点处理：当偏转角极小(接近0)时，喷管笔直向后
    if (delta_N_rad < 1e-5) {
        *omega1 = 0.0;
        *omega2 = 0.0;
        *omega3 = 0.0;
        return true;
    }

    // 3. 计算中间辅助几何变量
    double cos_theta_sq = cos(theta_rad) * cos(theta_rad);
    double sin_theta_sq = sin(theta_rad) * sin(theta_rad);

    // 严格按照论文公式(25)计算 omega2 的余弦值
    double cos_omega2 = (cos(delta_N_rad / 2.0) - cos_theta_sq) / sin_theta_sq;

    // 再次防止由于浮点数精度四舍五入导致的 acos(1.00001) 崩溃
    cos_omega2 = clamp(cos_omega2, -1.0, 1.0);

    // 4. 解算三级轴承的运动角度 (公式 25)
    double omega2_rad = acos(cos_omega2);
    double omega3_rad = -omega2_rad;

    // 计算 omega1
    double omega1_rad = -atan(tan(omega2_rad / 2.0) * cos(theta_rad)) + delta_Ny_rad;

    // 5. 将弧度结果还原转换为角度输出给执行机构（通过指针赋值）
    *omega1 = omega1_rad * 180.0 / M_PI;
    *omega2 = omega2_rad * 180.0 / M_PI;
    *omega3 = omega3_rad * 180.0 / M_PI;

    return true;
}