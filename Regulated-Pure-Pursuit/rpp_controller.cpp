#include "rpp_controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace rpp
{

// ===== Конструктор =====

RPPController::RPPController()
{
}

// ===== Configure =====

void RPPController::configure(
    const RPPParams& params,
    const uint8_t* costmap_data,
    int costmap_width, int costmap_height,
    float costmap_resolution,
    float costmap_origin_x, float costmap_origin_y)
{
    m_params = params;

    if (costmap_data && costmap_width > 0 && costmap_height > 0) {
        updateCostmap(costmap_data, costmap_width, costmap_height,
                      costmap_resolution, costmap_origin_x, costmap_origin_y);
    }
}

void RPPController::updateCostmap(
    const uint8_t* costmap_data,
    int width, int height,
    float resolution,
    float origin_x, float origin_y)
{
    m_collisionChecker.configure(
        costmap_data, width, height, resolution,
        origin_x, origin_y,
        3.0f,   // inscribed_radius
        4.2f);  // circumscribed_radius
}

void RPPController::setPath(const Path& path) {
    m_path = path;
}

void RPPController::reset()
{
    m_lastLinearVel = 0.0f;
    m_lastAngularVel = 0.0f;
    m_isRotatingToHeading = false;
}

// ===== Трансформация пути в систему координат робота =====

void RPPController::transformPathToRobotFrame(
    float robot_x, float robot_y, float robot_yaw)
{
    m_transformedPath.poses.clear();

    if (m_path.poses.empty()) return;

    // 1. Найти ближайшую точку к роботу
    int closest_idx = 0;
    float min_dist = std::numeric_limits<float>::max();

    for (size_t i = 0; i < m_path.poses.size(); ++i) {
        float dx = m_path.poses[i].position.x() - robot_x;
        float dy = m_path.poses[i].position.y() - robot_y;
        float dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = static_cast<int>(i);
        }
    }

    // 2. УДАЛЯЕМ пройденные точки
    if (closest_idx > 0) {
        m_path.poses.erase(m_path.poses.begin(), m_path.poses.begin() + closest_idx);
    }

    // 3. Трансформируем оставшиеся точки
    float cos_yaw = std::cos(-robot_yaw);
    float sin_yaw = std::sin(-robot_yaw);

    for (const auto& pose : m_path.poses) {
        float dx = pose.position.x() - robot_x;
        float dy = pose.position.y() - robot_y;

        float x_local = dx * cos_yaw - dy * sin_yaw;
        float y_local = dx * sin_yaw + dy * cos_yaw;

        Pose transformed;
        transformed.position.x() = x_local;
        transformed.position.y() = y_local;
        transformed.orientation = pose.orientation - robot_yaw;

        m_transformedPath.poses.push_back(transformed);
    }
}

// ===== Lookahead Distance =====

float RPPController::getLookAheadDistance(float current_speed)
{
    float lookahead_dist = m_params.lookahead_dist;

    if (m_params.use_velocity_scaled_lookahead_dist) {
        lookahead_dist = std::fabs(current_speed) * m_params.lookahead_time;
        lookahead_dist = std::clamp(lookahead_dist,
                                    m_params.min_lookahead_dist,
                                    m_params.max_lookahead_dist);
    }

    return lookahead_dist;
}

// ===== Поиск Lookahead Point =====

bool RPPController::findLookAheadPoint(
    float lookahead_dist, float& point_x, float& point_y)
{
    if (m_transformedPath.poses.empty()) return false;

    point_x = 0.0f;
    point_y = 0.0f;

    for (size_t i = 0; i < m_transformedPath.poses.size(); ++i) {
        const auto& pose = m_transformedPath.poses[i];
        float dist = std::hypot(pose.position.x(), pose.position.y());

        if (dist >= lookahead_dist) {
            if (i > 0) {
                const auto& prev = m_transformedPath.poses[i - 1];
                float prev_dist = std::hypot(prev.position.x(), prev.position.y());

                if (std::fabs(dist - prev_dist) > 1e-6f) {
                    float t = (lookahead_dist - prev_dist) / (dist - prev_dist);
                    t = std::clamp(t, 0.0f, 1.0f);
                    point_x = prev.position.x() + t * (pose.position.x() - prev.position.x());
                    point_y = prev.position.y() + t * (pose.position.y() - prev.position.y());
                } else {
                    point_x = pose.position.x();
                    point_y = pose.position.y();
                }
            } else {
                point_x = pose.position.x();
                point_y = pose.position.y();
            }
            return true;
        }
    }

    // Не нашли — берём последнюю точку
    const auto& last = m_transformedPath.poses.back();
    point_x = last.position.x();
    point_y = last.position.y();
    return true;
}

// ===== Вычисление кривизны =====

float RPPController::calculateCurvature(float lx, float ly) const
{
    float dist2 = lx * lx + ly * ly;
    if (dist2 > 0.001f) {
        return 2.0f * ly / dist2;
    }
    return 0.0f;
}

// ===== Эвристики регулирования скорости =====

float RPPController::curvatureConstraint(
    float raw_linear_vel, float curvature) const
{
    if (std::fabs(curvature) < 1e-6f) return raw_linear_vel;

    float radius = std::fabs(1.0f / curvature);
    float min_radius = m_params.regulated_linear_scaling_min_radius;

    if (radius < min_radius) {
        return raw_linear_vel * (1.0f - (std::fabs(radius - min_radius) / min_radius));
    }

    return raw_linear_vel;
}

float RPPController::costConstraint(
    float raw_linear_vel, float pose_cost) const
{
    if (pose_cost <= 0.0f || pose_cost >= 254.0f) return raw_linear_vel;

    const float inscribed_radius = 3.0f;
    float min_dist = (m_params.inflation_cost_scaling_factor * inscribed_radius -
                      std::log(pose_cost) + std::log(253.0f)) /
                     m_params.inflation_cost_scaling_factor;

    if (min_dist < m_params.cost_scaling_dist) {
        return raw_linear_vel *
               (m_params.cost_scaling_gain * min_dist / m_params.cost_scaling_dist);
    }

    return raw_linear_vel;
}

float RPPController::approachVelocityConstraint(
    float constrained_linear_vel) const
{
    if (m_transformedPath.poses.empty()) return constrained_linear_vel;

    // Оставшееся расстояние по пути
    float remaining_dist = 0.0f;
    for (size_t i = 1; i < m_transformedPath.poses.size(); ++i) {
        float dx = m_transformedPath.poses[i].position.x() -
                   m_transformedPath.poses[i - 1].position.x();
        float dy = m_transformedPath.poses[i].position.y() -
                   m_transformedPath.poses[i - 1].position.y();
        remaining_dist += std::hypot(dx, dy);
    }

    if (remaining_dist < m_params.approach_velocity_scaling_dist) {
        const auto& last = m_transformedPath.poses.back();
        float dist_to_goal = std::hypot(last.position.x(), last.position.y());
        float velocity_scaling = dist_to_goal / m_params.approach_velocity_scaling_dist;
        float approach_vel = constrained_linear_vel * velocity_scaling;

        approach_vel = std::max(approach_vel, m_params.min_approach_linear_velocity);
        return std::min(constrained_linear_vel, approach_vel);
    }

    return constrained_linear_vel;
}

void RPPController::applyConstraints(
    float curvature, float pose_cost,
    float& linear_vel, float& sign)
{
    float curvature_vel = linear_vel;
    float cost_vel = linear_vel;

    if (m_params.use_regulated_linear_velocity_scaling) {
        curvature_vel = curvatureConstraint(linear_vel, curvature);
    }

    if (m_params.use_cost_regulated_linear_velocity_scaling) {
        cost_vel = costConstraint(linear_vel, pose_cost);
    }

    linear_vel = std::min(cost_vel, curvature_vel);
    linear_vel = std::max(linear_vel, m_params.regulated_linear_scaling_min_speed);

    linear_vel = approachVelocityConstraint(linear_vel);

    linear_vel = std::clamp(std::fabs(linear_vel), 0.0f, m_params.max_linear_vel);
    linear_vel = sign * linear_vel;
}

// ===== Поворот к курсу =====

bool RPPController::shouldRotateToPath(float& angle_to_path, float x_vel_sign)
{
    if (m_transformedPath.poses.empty()) return false;

    const auto& carrot = m_transformedPath.poses.front();
    angle_to_path = std::atan2(carrot.position.y(), carrot.position.x());

    if (x_vel_sign < 0.0f) {
        angle_to_path = std::atan2(std::sin(angle_to_path + M_PI),
                                    std::cos(angle_to_path + M_PI));
    }

    return m_params.use_rotate_to_heading &&
           std::fabs(angle_to_path) > m_params.rotate_to_heading_min_angle;
}

bool RPPController::shouldRotateToGoalHeading(
    float robot_x, float robot_y,
    float goal_x, float goal_y, float goal_yaw,
    float current_speed)
{
    if (!m_params.use_rotate_to_heading) return false;
    if (m_transformedPath.poses.empty()) return false;

    float dist_to_goal = std::hypot(goal_x - robot_x, goal_y - robot_y);
    const float xy_goal_tolerance = 0.25f;

    if (dist_to_goal < xy_goal_tolerance) {
        return true;
    }

    return false;
}

void RPPController::rotateToHeading(
    float& linear_vel, float& angular_vel,
    float angle_to_path, float current_angular_vel, float dt)
{
    linear_vel = 0.0f;
    float sign = angle_to_path > 0.0f ? 1.0f : -1.0f;
    angular_vel = sign * m_params.rotate_to_heading_angular_vel;

    float min_feasible = current_angular_vel - m_params.max_angular_accel * dt;
    float max_feasible = current_angular_vel + m_params.max_angular_accel * dt;
    angular_vel = std::clamp(angular_vel, min_feasible, max_feasible);

    float max_vel_to_stop = std::sqrt(2.0f * m_params.max_angular_accel * std::fabs(angle_to_path));
    if (std::fabs(angular_vel) > max_vel_to_stop) {
        angular_vel = sign * max_vel_to_stop;
    }
}

// ===== Dynamic Window Pure Pursuit =====

void RPPController::computeDynamicWindowVelocities(
    float current_linear_vel, float current_angular_vel,
    float regulated_linear_vel, float curvature, float sign, float dt,
    float& out_linear_vel, float& out_angular_vel)
{
    float v_min, v_max;
    if (current_linear_vel > 0.0f) {
        v_max = std::min(current_linear_vel + m_params.max_linear_accel * dt,
                         m_params.max_linear_vel);
        v_min = std::max(current_linear_vel + m_params.max_linear_decel * dt,
                         m_params.min_linear_vel);
    } else {
        v_max = std::min(current_linear_vel - m_params.max_linear_decel * dt,
                         m_params.max_linear_vel);
        v_min = std::max(current_linear_vel - m_params.max_linear_accel * dt,
                         m_params.min_linear_vel);
    }

    if (sign >= 0.0f) {
        v_max = std::min(v_max, std::max(0.0f, regulated_linear_vel));
        v_min = std::max(v_min, 0.0f);
    } else {
        v_min = std::max(v_min, std::min(0.0f, regulated_linear_vel));
        v_max = std::min(v_max, 0.0f);
    }

    if (v_min > v_max) {
        if (v_min > std::max(0.0f, regulated_linear_vel)) {
            v_max = v_min;
        } else {
            v_min = v_max;
        }
    }

    if (std::fabs(curvature) < 1e-3f) {
        out_linear_vel = (sign >= 0.0f) ? v_max : v_min;
        out_angular_vel = 0.0f;
    } else {
        float w_min = curvature * v_min;
        float w_max = curvature * v_max;

        if (sign >= 0.0f) {
            out_linear_vel = v_max;
            out_angular_vel = w_max;
        } else {
            out_linear_vel = v_min;
            out_angular_vel = w_min;
        }
    }
}

// ===== Основной метод =====

RPPOutput RPPController::computeVelocityCommands(
    float robot_x, float robot_y, float robot_yaw,
    float current_linear_vel, float current_angular_vel,
    float dt)
{
    RPPOutput output;

    if (m_path.poses.empty()) {
        output.linear_vel = 0.0f;
        output.angular_vel = 0.0f;
        return output;
    }

    // Трансформируем путь в систему координат робота
    transformPathToRobotFrame(robot_x, robot_y, robot_yaw);

    // Lookahead distance
    float lookahead_dist = getLookAheadDistance(current_linear_vel);

    // Находим lookahead point
    float lookahead_x, lookahead_y;
    if (!findLookAheadPoint(lookahead_dist, lookahead_x, lookahead_y)) {
        output.linear_vel = 0.0f;
        output.angular_vel = 0.0f;
        return output;
    }

    m_lookaheadX = lookahead_x;
    m_lookaheadY = lookahead_y;

    // Кривизна
    float curvature = calculateCurvature(lookahead_x, lookahead_y);

    // Для curvature lookahead
    float regulation_curvature = curvature;
    if (m_params.use_fixed_curvature_lookahead) {
        float curve_x, curve_y;
        if (findLookAheadPoint(m_params.curvature_lookahead_dist, curve_x, curve_y)) {
            regulation_curvature = calculateCurvature(curve_x, curve_y);
        }
    }

    float x_vel_sign = 1.0f;
    if (m_params.allow_reversing) {
        x_vel_sign = lookahead_x >= 0.0f ? 1.0f : -1.0f;
    }

    float linear_vel = m_params.max_linear_vel;
    float angular_vel = 0.0f;

    // Определяем цель
    float goal_x = 0.0f, goal_y = 0.0f, goal_yaw = 0.0f;
    if (!m_path.poses.empty()) {
        const auto& goal = m_path.poses.back();
        goal_x = goal.position.x();
        goal_y = goal.position.y();
        goal_yaw = goal.orientation;
    }

    // Поворот к цели или пути
    float angle_to_path;
    if (shouldRotateToGoalHeading(robot_x, robot_y, goal_x, goal_y, goal_yaw,
                                   current_linear_vel)) {
        m_isRotatingToHeading = true;
        float angle_to_goal = 0.0f;
        if (!m_transformedPath.poses.empty()) {
            angle_to_goal = m_transformedPath.poses.back().orientation;
        }
        rotateToHeading(linear_vel, angular_vel, angle_to_goal,
                        current_angular_vel, dt);
    } else if (shouldRotateToPath(angle_to_path, x_vel_sign)) {
        m_isRotatingToHeading = true;
        rotateToHeading(linear_vel, angular_vel, angle_to_path,
                        current_angular_vel, dt);
    } else {
        m_isRotatingToHeading = false;

        // Применяем ограничения
        float pose_cost = m_collisionChecker.costAtPose(robot_x, robot_y);
        applyConstraints(regulation_curvature, pose_cost, linear_vel, x_vel_sign);

        if (!m_params.use_dynamic_window) {
            angular_vel = linear_vel * regulation_curvature;
        } else {
            computeDynamicWindowVelocities(
                current_linear_vel, current_angular_vel,
                linear_vel, regulation_curvature, x_vel_sign, dt,
                linear_vel, angular_vel);
        }
    }

    // Проверка коллизий
    if (m_params.use_collision_detection) {
        float carrot_dist = std::hypot(lookahead_x, lookahead_y);
        if (m_collisionChecker.isCollisionImminent(
                robot_x, robot_y, robot_yaw,
                linear_vel, angular_vel,
                carrot_dist,
                m_params.max_allowed_time_to_collision_up_to_carrot,
                dt)) {
            std::cerr << "[RPP] Collision imminent! Stopping." << std::endl;
            output.linear_vel = 0.0f;
            output.angular_vel = 0.0f;
            return output;
        }
    }

    // Формируем выход
    output.linear_vel = linear_vel;
    output.angular_vel = angular_vel;
    output.steering_angle = std::atan2(m_params.wheelbase * angular_vel,
                                        std::max(std::fabs(linear_vel), 0.1f));
    output.is_rotating_to_heading = m_isRotatingToHeading;

    m_lastLinearVel = linear_vel;
    m_lastAngularVel = angular_vel;

    return output;
}

}  // namespace rpp
