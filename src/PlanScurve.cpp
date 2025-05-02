#include "PlanScurve.h"

#define EPSILON 1e-6

static void calculate_phases(SCurveGenerator* gen) {
    const double j_max = gen->j_max;
    const double a_max = gen->a_max;
    const double v_max = gen->v_max;
    const double D = fabs(gen->q1 - gen->q0);
    
    // 计算各阶段理论持续时间
    double Tj1 = a_max / j_max;      // 达到最大加速度所需时间
    double Ta = 2*Tj1;               // 加速段总时间
    double Tv = (v_max - a_max*Tj1)/a_max; // 匀速段时间
    
    // 验证是否达到最大速度
    double S_acc = j_max * Tj1*Tj1*Tj1 + a_max*Tj1*Tj1; // 加速阶段位移
    if(2*S_acc > D) {  // 无法达到最大速度
        Tj1 = pow(D/(2*j_max), 1.0/3.0);
        Ta = 2*Tj1;
        Tv = 0;
    }

    // 计算各时间段
    gen->T[0] = Tj1;         // 加加速阶段
    gen->T[1] = Ta - 2*Tj1;  // 匀加速阶段
    gen->T[2] = Tj1;         // 减加速阶段
    gen->T[3] = Tv;          // 匀速阶段
    gen->T[4] = Tj1;         // 加减速阶段
    gen->T[5] = Ta - 2*Tj1;  // 匀减速阶段
    gen->T[6] = Tj1;         // 减减速阶段
    
    gen->t_total = Ta + Tv + Ta; // 总运动时间
}

void s_curve_init(SCurveGenerator* gen) {
    // 参数校验
    if(fabs(gen->q1 - gen->q0) < EPSILON) {
        gen->is_valid = false;
        return;
    }
    
    gen->dir = (gen->q1 > gen->q0) ? 1.0 : -1.0;
    calculate_phases(gen);
    
    // 初始化状态
    gen->t_current = 0;
    gen->q_current = gen->q0;
    gen->v_current = 0;
    gen->a_current = 0;
    gen->is_valid = true;
}

static double calculate_position(double t, const double T[7], double j_max, double dir) {
    double q = 0.0;
    const double T1 = T[0];      // 加加速阶段时间
    const double T2 = T[1];      // 匀加速阶段时间
    const double T3 = T[2];      // 减加速阶段时间
    const double Tv = T[3];      // 匀速阶段时间
    const double T4 = T[4];      // 加减速阶段时间
    const double T5 = T[5];      // 匀减速阶段时间
    const double T6 = T[6];      // 减减速阶段时间
    
    // 各阶段累计时间
    const double t1 = T1;
    const double t2 = t1 + T2;
    const double t3 = t2 + T3;
    const double t4 = t3 + Tv;
    const double t5 = t4 + T4;
    const double t6 = t5 + T5;
    
    // 第一阶段：加加速阶段 (j = +j_max)
    if(t < t1) {
        q = dir * (j_max * pow(t, 3)) / 6.0;
    }
    // 第二阶段：匀加速阶段 (j = 0)
    else if(t < t2) {
        double a_max = j_max * T1;  // 最大加速度
        q = dir * (
            (j_max * pow(T1, 3) / 6.0) + 
            (0.5 * a_max * pow(t - t1, 2)) +
            (0.5 * j_max * pow(T1, 2)) * (t - t1)
        );
    }
    // 第三阶段：减加速阶段 (j = -j_max)
    else if(t < t3) {
        double dt = t - t2;
        q = dir * (
            (j_max * pow(T1, 3) / 6.0) +
            (0.5 * j_max * pow(T1, 2) * T2) +
            (j_max * T1 * T2 * dt) +
            (0.5 * j_max * T1 * pow(dt, 2)) -
            (j_max * pow(dt, 3) / 6.0)
        );
    }
    // 第四阶段：匀速阶段 (j = 0, a = 0)
    else if(t < t4) {
        double v_cruise = j_max * T1 * (T1 + T2);
        q = dir * (
            (j_max * pow(T1, 3) / 6.0) +
            (0.5 * j_max * pow(T1, 2) * T2) +
            (j_max * T1 * T2 * T3) +
            (0.5 * j_max * T1 * pow(T3, 2)) -
            (j_max * pow(T3, 3) / 6.0) +
            v_cruise * (t - t3)
        );
    }
    // 第五阶段：加减速阶段 (j = -j_max)
    else if(t < t5) {
        double dt = t - t4;
        double v_cruise = j_max * T1 * (T1 + T2);
        q = dir * (
            (j_max * pow(T1, 3) / 6.0) +
            (0.5 * j_max * pow(T1, 2) * T2) +
            (j_max * T1 * T2 * T3) +
            (0.5 * j_max * T1 * pow(T3, 2)) -
            (j_max * pow(T3, 3) / 6.0) +
            v_cruise * Tv -
            (j_max * pow(dt, 3) / 6.0)
        );
    }
    // 第六阶段：匀减速阶段 (j = 0)
    else if(t < t6) {
        double dt = t - t5;
        double a_decel = -j_max * T4;  // 减速阶段加速度
        q = dir * (
            (j_max * pow(T1, 3) / 6.0) +
            (0.5 * j_max * pow(T1, 2) * T2) +
            (j_max * T1 * T2 * T3) +
            (0.5 * j_max * T1 * pow(T3, 2)) -
            (j_max * pow(T3, 3) / 6.0) +
            (j_max * T1 * (T1 + T2) * Tv) -
            (j_max * pow(T4, 3) / 6.0) +
            (0.5 * a_decel * pow(dt, 2)) +
            (a_decel * T4) * dt
        );
    }
    // 第七阶段：减减速阶段 (j = +j_max)
    else {
        double dt = t - t6;
        q = dir * (
            (j_max * pow(T1, 3) / 6.0) +
            (0.5 * j_max * pow(T1, 2) * T2) +
            (j_max * T1 * T2 * T3) +
            (0.5 * j_max * T1 * pow(T3, 2)) -
            (j_max * pow(T3, 3) / 6.0) +
            (j_max * T1 * (T1 + T2) * Tv) -
            (j_max * pow(T4, 3) / 6.0) +
            (0.5 * (-j_max * T4) * pow(T5, 2)) +
            (-j_max * T4 * T5) * T6 +
            (0.5 * j_max * pow(dt, 2) * T6) -
            (j_max * pow(dt, 3) / 6.0)
        );
    }
    
    return q;
}
bool s_curve_update(SCurveGenerator* gen, double dt) {
    if(!gen->is_valid) return false;
    
    gen->t_current += dt;
    if(gen->t_current > gen->t_total) {
        gen->t_current = gen->t_total;
    }
    
    // 计算各阶段参数
    double t = gen->t_current;
    gen->q_current = gen->q0 + calculate_position(t, gen->T, gen->j_max, gen->dir);
    
    // 计算速度/加速度（根据阶段选择方程）
    if(t < gen->T[0]) {
        gen->a_current = gen->dir * gen->j_max * t;
        gen->v_current = gen->dir * gen->j_max * t*t / 2;
    }
    // ... 其他阶段
    
    return (gen->t_current < gen->t_total);
}