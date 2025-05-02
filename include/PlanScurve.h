#ifndef S_CURVE_H
#define S_CURVE_H

#include <math.h>
#include <stdbool.h>

typedef struct {
    /* 用户配置参数 */
    double q0;          // 起始位置
    double q1;          // 目标位置
    double v_max;       // 最大速度 (m/s or rad/s)
    double a_max;       // 最大加速度 (m/s² or rad/s²)
    double j_max;       // 最大加加速度 (m/s³ or rad/s³)
    
    /* 内部状态 */
    double T[7];        // 七个阶段的时间段
    double t_total;     // 总运动时间
    double dir;         // 运动方向(1或-1)
    bool is_valid;      // 轨迹是否有效
    
    /* 实时状态 */
    double t_current;   // 当前时间
    double q_current;   // 当前位置
    double v_current;   // 当前速度
    double a_current;   // 当前加速度
} SCurveGenerator;

void s_curve_init(SCurveGenerator* gen);
bool s_curve_update(SCurveGenerator* gen, double dt);

#endif