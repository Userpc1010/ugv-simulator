#ifndef RPP_CONTROLLER_HPP
#define RPP_CONTROLLER_HPP

#include "datatypes_smac_planner.h"
#include "rpp_collision_checker.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cstdint>
#include <cmath>

namespace rpp
{

// ===== Выход контроллера =====
struct RPPOutput
{
    float linear_vel = 0.0f;      // v_cmd
    float angular_vel = 0.0f;     // w_cmd
    float steering_angle = 0.0f;  // для Ackermann
    bool is_rotating_to_heading = false;
};

// ===== Параметры RPP =====
struct RPPParams
{
    // Скорости
    float max_linear_vel = 20.0f;
    float min_linear_vel = -10.0f;
    float max_angular_vel = 0.25f;
    float min_angular_vel = -0.25f;

    // Ускорения (для Dynamic Window)
    float max_linear_accel = 9.0f;
    float max_linear_decel = -9.0f;
    float max_angular_accel = 1.2f;
    float max_angular_decel = -1.2f;

    // Lookahead
    float lookahead_dist = 10.0f;
    float min_lookahead_dist = 5.0f;
    float max_lookahead_dist = 25.0f;
    float lookahead_time = 3.5f;
    bool use_velocity_scaled_lookahead_dist = true;

    // Регуляция скорости
    bool use_regulated_linear_velocity_scaling = true;
    bool use_cost_regulated_linear_velocity_scaling = false;
    float regulated_linear_scaling_min_radius = 16.0f;
    float regulated_linear_scaling_min_speed = 1.0f;

    // Подход к цели
    float min_approach_linear_velocity = 1.5f;
    float approach_velocity_scaling_dist = 15.0f;

    // Поворот к цели/пути
    bool use_rotate_to_heading = false;
    float rotate_to_heading_angular_vel = 0.25f;
    float rotate_to_heading_min_angle = 0.785f;

    // Коллизия
    bool use_collision_detection = true;
    float max_allowed_time_to_collision_up_to_carrot = 1.0f;
    float min_distance_to_obstacle = -1.0f;

    // Cost scaling
    float cost_scaling_dist = 2.0f;
    float cost_scaling_gain = 1.0f;
    float inflation_cost_scaling_factor = 3.0f;

    // Фиксированный curvature lookahead
    bool use_fixed_curvature_lookahead = false;
    float curvature_lookahead_dist = 5.0f;
    bool interpolate_curvature_after_goal = false;

    // Dynamic Window Pure Pursuit
    bool use_dynamic_window = false;

    // Ackermann
    bool allow_reversing = true;
    float wheelbase = 7.107f;
};

/**
 * @brief Regulated Pure Pursuit Controller
 */
class RPPController
{
public:
    RPPController();

    void configure(
        const RPPParams& params,
        const uint8_t* costmap_data = nullptr,
        int costmap_width = 0, int costmap_height = 0,
        float costmap_resolution = 1.0f,
        float costmap_origin_x = 0.0f, float costmap_origin_y = 0.0f);

    void setPath(const Path& path);

    RPPOutput computeVelocityCommands(
        float robot_x, float robot_y, float robot_yaw,
        float current_linear_vel, float current_angular_vel,
        float dt);

    void updateCostmap(
        const uint8_t* costmap_data,
        int width, int height,
        float resolution,
        float origin_x, float origin_y);

    void reset();

    void getLookaheadPoint(float& lx, float& ly) const {
        lx = m_lookaheadX; ly = m_lookaheadY;
    }

private:
    RPPParams m_params;
    CollisionChecker m_collisionChecker;
    Path m_path;
    Path m_transformedPath;

    float m_lookaheadX = 0.0f;
    float m_lookaheadY = 0.0f;

    float m_lastLinearVel = 0.0f;
    float m_lastAngularVel = 0.0f;
    bool m_isRotatingToHeading = false;

    // Вспомогательные методы
    void transformPathToRobotFrame(float robot_x, float robot_y, float robot_yaw);
    float getLookAheadDistance(float current_speed);
    bool findLookAheadPoint(float lookahead_dist, float& point_x, float& point_y);
    float calculateCurvature(float lx, float ly) const;

    float curvatureConstraint(float raw_linear_vel, float curvature) const;
    float costConstraint(float raw_linear_vel, float pose_cost) const;
    float approachVelocityConstraint(float constrained_linear_vel) const;
    void applyConstraints(float curvature, float pose_cost, float& linear_vel, float& sign);

    bool shouldRotateToPath(float& angle_to_path, float x_vel_sign);
    bool shouldRotateToGoalHeading(
        float robot_x, float robot_y,
        float goal_x, float goal_y, float goal_yaw,
        float current_speed);
    void rotateToHeading(
        float& linear_vel, float& angular_vel,
        float angle_to_path, float current_angular_vel, float dt);

    void computeDynamicWindowVelocities(
        float current_linear_vel, float current_angular_vel,
        float regulated_linear_vel, float curvature, float sign, float dt,
        float& out_linear_vel, float& out_angular_vel);
};

}  // namespace rpp

#endif  // RPP_CONTROLLER_HPP
