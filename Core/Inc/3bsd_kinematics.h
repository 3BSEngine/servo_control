//
// Created by hp_cc on 2026/7/13.
//

#ifndef UART_CONTROL_NEW_SERVO_ALGORITHM_3BSD_KINEMATICS_H
#define UART_CONTROL_NEW_SERVO_ALGORITHM_3BSD_KINEMATICS_H

#endif //UART_CONTROL_NEW_SERVO_ALGORITHM_3BSD_KINEMATICS_H
#ifndef __3BSD_KINEMATICS_H
#define __3BSD_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// 定义常量 PI（C 语言没有 M_PI 时手动定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 三轴承推力矢量喷管(3BSD)运动学逆解算函数
 * @param delta_N   [输入] 期望的喷管整体偏转角大小 (单位: 度, Range: 0 ~ 96度)
 * @param delta_Ny  [输入] 期望的偏转方向角 (单位: 度)
 * @param theta     [输入] 喷管截面倾角 (单位: 度, 选取 26.25)
 * @param omega1    [输出] 第一级喷管绕发动机轴线的转动角度 (单位: 度)
 * @param omega2    [输出] 第二级喷管相对第一级喷管的转动角度 (单位: 度)
 * @param omega3    [输出] 第三级喷管相对第二级喷管的转动角度 (单位: 度)
 * @return bool     解算是否成功
 */
bool solveInverseKinematics(double delta_N, double delta_Ny, double theta,
                            double* omega1, double* omega2, double* omega3);

#ifdef __cplusplus
}
#endif

#endif /* __3BSD_KINEMATICS_H */