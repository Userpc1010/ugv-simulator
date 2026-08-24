#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include <Eigen/Dense>
#include <algorithm>
#include "simulation.h"

class CarController {
public:
    CarController();

    void setCommands(float w_cmd, float v_cmd);
    control_vec update(const state_vec& state, float dt);
    void reset();

    // ПИД-коэффициенты
    float kp_yaw   = 0.5f;
    float ki_yaw   = 0.01f;
    float kd_yaw   = 0.1f;

    float kp_speed              = 0.08f;      // 0.1 * 10 м/с ошибки = 1.0 (полный газ при старте)
    float kd_speed              = 0.02f;      // D гасит перелёт
    float deriv_threshold_speed = 0.25f;       // м/с² — ниже этого считаем шумом

    float ki_speed = 0.001f;     // I медленно убирает статическую ошибку
    float ff_speed = 0.005f;     // FF: 0.05 * 10 м/с = 0.5 (пол-газа на крейсерской)

    float kp_brake              = 0.05f;
    float ki_brake              = 0.001f;
    float kd_brake              = 0.01f;
    float deriv_threshold_brake = 0.25f;

    // Геометрия
    float max_steer       = 0.523599f;  // физический предел сервы (30°)
    float wheelbase       = 7.107f;      // колёсная база
    float max_centrifugal = 5.0f;      // макс. допустимое боковое ускорение (м/с²) ≈0.5g
    float safe_max_coff   = 1.25f;      // Буфер для ограничения угла поврота колеса по центробежной силе

    float m_blendStartSpeed = 10.0f;
    float m_blendEndSpeed = 15.0f;

private:

    float m_w_cmd = 0.0f;
    float m_v_cmd = 0.0f;

    float m_integral_yaw = 0.0f;
    float m_prev_error_yaw = 0.0f;
    float m_integral_speed = 0.0f;
    float m_prev_v_cmd = 0.0f;

    float Last_Error_Motor = 0.0f;
    float Last_Error_Brake = 0.0f;
    float last_Integral_Motor = 0.0f;
    float last_Integral_Brake = 0.0f;

    float filVal = 0.0f;
    float k = 0.01f;

    // Накопитель угла для плавного возврата в ноль (как в main.c)
    float m_current_steer_angle = 0.0f;

    static constexpr float STEER_SPEED = 2.25f;  // скорость вращения руля (рад/с)

    static constexpr float YAW_INTEGRAL_LIMIT   = 0.5f;
    static constexpr float SPEED_INTEGRAL_LIMIT = 1.0f;
    static constexpr float BRAKE_INTEGRAL_LIMIT = 1.0f;

    // Вычисление динамического лимита угла по текущей скорости
    float getSafeMaxSteer(float v_fwd) const;
    float calculate (float A2, float B2, float C2);
    float expRunningAverage(float newVal);
    float curvatureConstraint(float linear_vel, float curvature, float min_radius);
};

#endif // CAR_CONTROLLER_H
