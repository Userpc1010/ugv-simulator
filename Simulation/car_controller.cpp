#include "car_controller.h"
#include <cmath>

CarController::CarController() { reset(); }

float CarController::getSafeMaxSteer(float v_fwd) const {
    if (std::fabs(v_fwd) < 1.0f) return max_steer;  // на малых скоростях полный угол

    float v2 = v_fwd * v_fwd;
    float safe_tan = (max_centrifugal * wheelbase) / v2;
    return std::min(max_steer, std::atan(safe_tan));
}

float CarController::calculate(float A2, float B2, float C2)
{
    uint8_t A2_gt_B2 = (A2 > B2) ? 1 : 0;
       uint8_t C2_gt_A2 = (C2 - A2 > 0) ? 1 : 0;
       uint8_t A2_lt_0 = (A2 < 0) ? 1 : 0;
       uint8_t B2_lt_0 = (B2 < 0) ? 1 : 0;
       uint8_t C2_lt_0 = (C2 < 0) ? 1 : 0;
       uint8_t B2_eq_A2 = (std::fabs(B2 - A2) < 1e-6f) ? 1 : 0;

       if (A2_gt_B2) {
           return (C2_gt_A2) ? ((A2_lt_0 && B2_lt_0) ? C2 : -B2) : 0;
       } else if (A2_lt_0) {
           return (B2_lt_0 && C2_lt_0) ? 0 : (B2_eq_A2 ? 0 : B2);
       } else {
           return -C2;
       }
}

float CarController::expRunningAverage(float newVal)
{
  filVal += (newVal - filVal) * k;
  return filVal;
}

float CarController::curvatureConstraint(float linear_vel, float curvature, float min_radius)
{
    // curvature = w / v (или из геометрии)
    // min_radius = wheelbase / tan(safe_max)

    if (curvature < 1e-6f) return linear_vel;  // прямой путь

    float radius = 1.0f / curvature;
    float min_radius_safe = min_radius;

    if (radius < min_radius_safe) {
        // Скорость падает пропорционально отношению радиусов
        float scale = radius / min_radius_safe;
        return linear_vel * scale;
    }
    return linear_vel;
}

void CarController::setCommands(float w_cmd, float v_cmd) {
    m_w_cmd = w_cmd;
    m_v_cmd = v_cmd;
}

void CarController::reset() {
    m_integral_yaw = 0.0f;
    m_prev_error_yaw = 0.0f;
    m_integral_speed = 0.0f;
    m_current_steer_angle = 0.0f;
}

control_vec CarController::update(const state_vec& state, float dt) {
    control_vec ctrl;
    if (dt <= 0.0f) dt = 0.005f;

    // Текущие величины
    float current_yaw = state.angVel.y();
    Eigen::Vector3f local_vel = state.ori.conjugate() * state.vel;
    float v_fwd = local_vel.z();

    // ===== РЕГУЛЯТОР УГЛОВОЙ СКОРОСТИ =====

    float safe_max = getSafeMaxSteer(v_fwd);
    float safe_max_angle = safe_max * safe_max_coff;

    float abs_v = fabs(v_fwd);

    float ads_s = fabs(m_w_cmd);

    if (abs_v > 0.1f) {

        float blend = ads_s / 6.5f;

        // Взвешенный знаменатель
        float v_denom = (1.0f - blend) * ads_s + blend * abs_v;

        float steer_rad = atan2(m_w_cmd * wheelbase, v_denom);

        steer_rad = std::clamp(steer_rad, -safe_max_angle, safe_max_angle);

        if (steer_rad >  max_steer) steer_rad =  max_steer;
        if (steer_rad < -max_steer) steer_rad = -max_steer;

        m_current_steer_angle = steer_rad ;

    }
        ctrl.steerAngle = m_current_steer_angle;

    // ===== ОГРАНИЧЕНИЕ СКОРОСТИ ПО КРИВИЗНЕ =====
    if (m_v_cmd >= 0.0f) {
        float curvature = std::fabs(m_w_cmd) / std::max(std::fabs(v_fwd), 1.0f);
        float min_radius = wheelbase / std::tan(safe_max);
        float curvature_vel = curvatureConstraint(m_v_cmd, curvature, min_radius);

        float v_limited = std::min(m_v_cmd, curvature_vel);
        float min_speed = 1.0f;
        v_limited = std::max(v_limited, min_speed);

        m_v_cmd = v_limited;
    }

    // ===== РЕГУЛЯТОР ЛИНЕЙНОЙ СКОРОСТИ =====
    float feed_forward_motor = m_v_cmd * ff_speed;

    float Error_Motor = m_v_cmd - v_fwd;
    float Error_Motor_ABS = std::abs(Error_Motor);

    float Delta_Motor = (Error_Motor - Last_Error_Motor) / dt;
    Last_Error_Motor = Error_Motor;

    if (Delta_Motor > deriv_threshold_speed) Delta_Motor -= deriv_threshold_speed;
    if (Delta_Motor < -deriv_threshold_speed) Delta_Motor += deriv_threshold_speed;

    float Integral_Motor = 0.0f;

    bool sign_changed = (m_v_cmd > 0.1f && m_prev_v_cmd < -0.1f) ||
                        (m_v_cmd < -0.1f && m_prev_v_cmd > 0.1f);

    if (Error_Motor_ABS > 0.5f && Error_Motor_ABS < 3.0f && std::fabs(m_v_cmd) > 0.5f) {
        Integral_Motor = last_Integral_Motor + Error_Motor * dt;
    } else {
        last_Integral_Motor *= 0.9f;
    }

    if (std::fabs(m_v_cmd) < 0.01f || sign_changed) {
        last_Integral_Motor = 0.0f;
    }

    if (Integral_Motor > SPEED_INTEGRAL_LIMIT) Integral_Motor = SPEED_INTEGRAL_LIMIT;
    if (Integral_Motor < -SPEED_INTEGRAL_LIMIT) Integral_Motor = -SPEED_INTEGRAL_LIMIT;

    last_Integral_Motor = Integral_Motor;
    m_prev_v_cmd = m_v_cmd;

    float pid_output_motor = kp_speed * Error_Motor + Delta_Motor * kd_speed + Integral_Motor * ki_speed;
    ctrl.throttle = (pid_output_motor + feed_forward_motor);

    if (ctrl.throttle > 1.0f) ctrl.throttle = 1.0f;
    if (ctrl.throttle < -1.0f) ctrl.throttle = -1.0f;

    if (std::fabs(m_v_cmd) < 1e-6f) {
        ctrl.throttle = 0;
        last_Integral_Motor = 0.0f;
        Integral_Motor = 0.0f;
    }

    if ((ctrl.throttle >= 1.0f && Error_Motor > 0) ||
        (ctrl.throttle <= -1.0f && Error_Motor < 0)) {
        last_Integral_Motor = Integral_Motor;
    }

    // ===== ТОРМОЗ =====
    float Error_Brake = expRunningAverage(calculate(m_v_cmd, v_fwd, Error_Motor));

    float Delta_Brake = (Error_Brake - Last_Error_Brake) / dt;
    Last_Error_Brake = Error_Brake;

    if (Delta_Brake > deriv_threshold_brake) Delta_Brake -= deriv_threshold_brake;
    if (Delta_Brake < -deriv_threshold_brake) Delta_Brake += deriv_threshold_brake;

    float Integral_Brake = 0.0f;

    if (Error_Motor_ABS > 0.5f && Error_Motor_ABS < 3.0f) {
        Integral_Brake = last_Integral_Brake + Error_Brake * dt;
    } else {
        last_Integral_Brake *= 0.9f;
    }

    if (Integral_Brake > BRAKE_INTEGRAL_LIMIT) Integral_Brake = BRAKE_INTEGRAL_LIMIT;
    if (Integral_Brake < -BRAKE_INTEGRAL_LIMIT) Integral_Brake = -BRAKE_INTEGRAL_LIMIT;

    last_Integral_Brake = Integral_Brake;

    ctrl.brake = (kp_brake * Error_Brake + Delta_Brake * kd_brake + Integral_Brake * ki_brake);

    if (ctrl.brake > 1.0f) ctrl.brake = 1.0f;
    if (ctrl.brake < 0.0f) ctrl.brake = 0.0f;

    if (Error_Brake < 1.0f) {
        ctrl.brake = 0.0f;
        last_Integral_Brake = 0.0f;
        Integral_Brake = 0.0f;
    }

    if (ctrl.brake >= 1.0f && Error_Brake > 0.0f) {
        last_Integral_Brake = Integral_Brake;
    }

    if (std::fabs(m_v_cmd) < 0.01f) m_integral_speed = 0.0f;
    if (std::fabs(m_w_cmd) < 0.001f) m_integral_yaw = 0.0f;

    return ctrl;
}
