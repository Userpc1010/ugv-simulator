#include "simulation.h"
#include <algorithm>
#include <QDebug>
#include <QString>

Simulation_Car::Simulation_Car() {
    // Начальные координаты
    data.pos = Eigen::Vector3f(-449.743f, 23.574f, 417.968f);
    data.vel = Eigen::Vector3f::Zero();
    data.angVel = Eigen::Vector3f::Zero();
    data.ori = Eigen::Quaternionf::Identity();
    data.steerAngle = 0.0f;
    data.wheelPos.setZero();


   // База (L): Разница между Z переднего и заднего колеса
   cfg.wheelbase = std::abs(cfg.wheelOffsets[0].z() - cfg.wheelOffsets[2].z());

   // Колея (W): Разница между X левого и правого колеса
   cfg.trackWidth = std::abs(cfg.wheelOffsets[0].x() - cfg.wheelOffsets[1].x());

   // Тот самый коэффициент Аккермана (W / 2L)
  if (cfg.wheelbase > 0.1f) {
   cfg.ackermannFactor = cfg.trackWidth / (2.0f * cfg.wheelbase);
  }

 // updateDampingSpring();

}

void Simulation_Car::updateDampingSpring()
{
    float mass_per_wheel = cfg.mass / 4.0f; // 550 кг
    cfg.damping_rebound = 2.0f * sqrtf(mass_per_wheel * cfg.k_spring);

    // На сжатие можно сделать 0.5-0.7 от критического
    cfg.damping_compression = cfg.damping_rebound * 0.5f;
}

void Simulation_Car::reset() {
    data.pos = Eigen::Vector3f(-449.743f, 23.574f, 417.968f);
    data.vel = Eigen::Vector3f::Zero();
    data.accel = Eigen::Vector3f::Zero();

    data.ori = Eigen::Quaternionf::Identity();
    data.angVel = Eigen::Vector3f::Zero();

    data.steerAngle = 0.0f;
    data.wheelPos.setZero();
    data.wheelVel.setZero();
    data.wheelAngularVel.setZero();
    data.wheelRotation.setZero();
}

state_vec Simulation_Car::update(const control_vec& ctrl, float dt) {
    const int subSteps = 10;
    float subDt = dt / subSteps;

    data.steerAngle += std::clamp( (ctrl.steerAngle - data.steerAngle) * 10.0f * dt, -cfg.max_sterr_speed * dt, cfg.max_sterr_speed * dt);

    for (int step = 0; step < subSteps; ++step) {
        Eigen::Matrix3f rotMat = data.ori.toRotationMatrix();
        Eigen::Matrix3f invRotMat = rotMat.transpose();

        // Силы в МИРОВЫХ координатах
        Eigen::Vector3f totalForceWorld(0, -gravity_v * cfg.mass, 0);

        // Моменты в ЛОКАЛЬНЫХ координатах
        Eigen::Vector3f totalTorqueLocal = Eigen::Vector3f::Zero();

        // Скорости в локальной системе
        Eigen::Vector3f local_V = invRotMat * data.vel;
        Eigen::Vector3f local_AngVel = data.angVel;

        Eigen::DiagonalMatrix<float, 3> I(cfg.inertia);
        Eigen::DiagonalMatrix<float, 3> invI(1.0f/cfg.inertia.x(), 1.0f/cfg.inertia.y(), 1.0f/cfg.inertia.z());

        // Аэродинамика (в мировой)
        float speed = data.vel.norm();
        if (speed > 0.1f) {
            totalForceWorld += -0.5f * 1.225f * speed * data.vel * 0.35f * 2.2f;
        }

        Eigen::Vector3f carUp_world = rotMat * Eigen::Vector3f::UnitY();

        for (size_t i = 0; i < cfg.wheelOffsets.size(); ++i) {
            const Eigen::Vector3f& local_offset = cfg.wheelOffsets[i];

            // === ГЕОМЕТРИЯ ===
            Eigen::Vector3f world_r = rotMat * local_offset;
            Eigen::Vector3f p_wheel = data.pos + world_r;  // точка крепления

            // === СКОРОСТЬ в локальной СК ===
            Eigen::Vector3f local_v_mount = local_V + local_AngVel.cross(local_offset);
            Eigen::Vector3f v_wheel = rotMat * local_v_mount;

            // === ВЫСОТА ЗЕМЛИ ===
            float h_pixel = (getGroundHeight) ? getGroundHeight(p_wheel.x(), p_wheel.z()) : 0.0f;
            Eigen::Vector3f groundNormal = (getGroundNormal) ? getGroundNormal(p_wheel.x(), p_wheel.z()) : Eigen::Vector3f::UnitY();

            float dist = p_wheel.y() - h_pixel;

            if (dist < cfg.susp_length && carUp_world.dot(groundNormal) > 0.1f) {

                data.wheelPos[i] = cfg.susp_length - dist;
                float v_rel = v_wheel.dot(groundNormal);

                // === ПОДВЕСКА ===
                float damping_coeff = (v_rel >= 0.0f) ? cfg.damping_compression : cfg.damping_rebound;
                float F_susp_raw = data.wheelPos[i] * cfg.k_spring * (1.0f + data.wheelPos[i] * 5.0f) - v_rel * damping_coeff;

                const float MAX_F = 4.0f * cfg.mass * gravity_v;
                float f_z = MAX_F * std::tanh(std::max(0.0f, F_susp_raw) / MAX_F);

                Eigen::Vector3f F_susp = groundNormal * f_z;
                totalForceWorld += F_susp;
                totalTorqueLocal += local_offset.cross(invRotMat * F_susp);

                // === ТРЕНИЕ И ТЯГА ===
                Eigen::Vector3f n_g_local = invRotMat * groundNormal;

                // Направление вперёд в локальной СК с Аккерманом
                Eigen::Vector3f wheel_fwd_local = Eigen::Vector3f::UnitZ();
                if (local_offset.z() > 0) {  // передние колёса
                    float side_sign = (local_offset.x() > 0) ? 1.0f : -1.0f;
                    float ackermann_angle = data.steerAngle / (1.0f + side_sign * cfg.ackermannFactor * data.steerAngle);
                    wheel_fwd_local = Eigen::AngleAxisf(ackermann_angle, Eigen::Vector3f::UnitY()) * Eigen::Vector3f::UnitZ();
                }

                // Проекция на плоскость земли
                Eigen::Vector3f fwd_local = (wheel_fwd_local - n_g_local * wheel_fwd_local.dot(n_g_local)).normalized();
                Eigen::Vector3f side_local = n_g_local.cross(fwd_local).normalized();

                // Скорости в локальной СК для трения
                float v_fwd = local_v_mount.dot(fwd_local);
                float v_side = local_v_mount.dot(side_local);

                // === ТЯГА (сила от двигателя) ===
//                float f_drive = 0.0f;
//                float drive_req = ctrl.throttle * cfg.throttle;

                // === ВРАЩЕНИЕ КОЛЁС ===
                float wheel_I = 0.5f * cfg.wheel_mass * cfg.wheel_radius * cfg.wheel_radius;

                // Продольное скольжение
                float v_tire = data.wheelAngularVel[i] * cfg.wheel_radius;
                const float EPS = 1e-6f;

                // Используем максимальную из скоростей, чтобы избежать деления на ноль
                float denom = std::max(std::abs(v_fwd), 1.0f);  // минимум 0.5 м/с (~1.8 км/ч)
                float slip_ratio = (v_tire - v_fwd) / denom;

                // Угол бокового скольжения
                float slip_angle = std::atan2(v_side, denom);

                // Комбинированное скольжение для Пасейки
                float total_slip = std::sqrt(slip_ratio * slip_ratio + slip_angle * slip_angle);
                //float pacejka_val = pacejka(total_slip);
                PacejkaParams pj = cfg.pacejka_default;
                if (getGroundPacejka) {
                    pj = getGroundPacejka(p_wheel.x(), p_wheel.z());
                }
                float pacejka_val = pacejka_with_params(total_slip, pj);
                //float max_f = f_z * cfg.mu;
                float zone_mu = getGroundMu ? getGroundMu(p_wheel.x(), p_wheel.z(), data.ori) : cfg.mu;
                float max_f = f_z * zone_mu;

                // Распределение силы трения по направлениям
                float f_x_real = pacejka_val * max_f * (slip_ratio / (total_slip + EPS));
                float f_side_real = -pacejka_val * max_f * (slip_angle / (total_slip + EPS));

                // Ограничение тяги по кругу трения
//                float f_x_max = std::sqrt(std::max(0.0f, pow2(max_f) - pow2(f_side_real)));
//                f_drive = std::clamp(drive_req, -f_x_max, f_x_max);

                // Тяговый момент
                //float drive_torque = ctrl.throttle * cfg.throttle * cfg.wheel_radius;
                // Тяговый момент от электромотора
                float motor_torque = getMotorTorque(data.wheelAngularVel[i], ctrl.throttle);
                float drive_torque = motor_torque * cfg.gear_ratio * cfg.drivetrain_efficiency;

                // Ограничение тяги по сцеплению (max_f уже с zone_mu)
                float max_drive_torque = max_f * cfg.wheel_radius;
                drive_torque = std::clamp(drive_torque, -max_drive_torque, max_drive_torque);

                // Тормозной момент
//                float brake_torque = 0.0f;
//                if (ctrl.brake > 0.001f) {
//                    brake_torque = -std::copysign(ctrl.brake * cfg.brake, data.wheelAngularVel[i]);
//                }
                // Тормозной момент ограничен сцеплением
                float brake_torque = 0.0f;
                if (ctrl.brake > 0.001f) {
                    float requested_brake = ctrl.brake * cfg.brake; // cfg.brake теперь можно удалить, но оставь для совместимости
                    float max_brake_torque = max_f * cfg.wheel_radius;

                    brake_torque = -std::copysign(std::min(requested_brake, max_brake_torque),
                                                   data.wheelAngularVel[i]);

                    // Заблокированное колесо не тормозит
                    if (std::abs(data.wheelAngularVel[i]) < 0.1f) {
                        brake_torque = 0.0f;
                    }
                }

                // Сопротивление качению
                //float rolling_torque = -data.wheelAngularVel[i] * cfg.rolling_resistance;
                float zone_rr = getGroundRR ? getGroundRR(p_wheel.x(), p_wheel.z(), data.ori) : cfg.rolling_resistance;
                float rolling_torque = -data.wheelAngularVel[i] * zone_rr;

                // В трясине: трение покоя (не зависит от скорости вращения колеса)
                if (zone_mu < 0.05f) {
                    // Сила трения покоя = 60% веса машины
                    float static_friction = 0.6f * cfg.mass * gravity_v / 4.0f;  // на колесо

                    // Момент от трения покоя
                    float static_torque = static_friction * cfg.wheel_radius;

                    // Применяем против направления вращения
                    if (std::abs(data.wheelAngularVel[i]) > 0.01f) {
                        rolling_torque = -std::copysign(static_torque, data.wheelAngularVel[i]);
                    } else {
                        rolling_torque = 0;
                    }
                }

                // Момент от продольной силы трения
                float friction_torque = -f_x_real * cfg.wheel_radius;

                // Межколёсный дифференциал
                float diff_torque = 0.0f;
                if (local_offset.z() > 0) {  // передняя ось
                    float front_avg = (data.wheelAngularVel[0] + data.wheelAngularVel[1]) * 0.5f;
                    diff_torque = (front_avg - data.wheelAngularVel[i]) * cfg.diff_lock;
                } else {  // задняя ось
                    float rear_avg = (data.wheelAngularVel[2] + data.wheelAngularVel[3]) * 0.5f;
                    diff_torque = (rear_avg - data.wheelAngularVel[i]) * cfg.diff_lock;
                }

                // Суммарный момент и интегрирование
                float total_wheel_torque = drive_torque + brake_torque + rolling_torque + friction_torque + diff_torque;
                data.wheelAngularVel[i] += (total_wheel_torque / wheel_I) * subDt;
                data.wheelRotation[i] += data.wheelAngularVel[i] * subDt;

                // === СИЛЫ ТРЕНИЯ В ЛОКАЛЬНОЙ СК ===
                Eigen::Vector3f F_friction_local = fwd_local * f_x_real + side_local * f_side_real;

                // Торможение (добавляем к силам трения)
//                if (ctrl.brake > 0.001f) {
//                    Eigen::Vector3f v_plane_local = local_v_mount - n_g_local * local_v_mount.dot(n_g_local);
//                    if (v_plane_local.squaredNorm() > 0.01f) {
//                        F_friction_local += -v_plane_local.normalized() * ctrl.brake * cfg.brake;
//                    }
//                }

                // Добавляем силы в мировую
                totalForceWorld += rotMat * F_friction_local;
                totalTorqueLocal += local_offset.cross(F_friction_local);
            }
        }

        // Вязкое сопротивление поверхности (действует на кузов)
        float drag_coef = 0.0f;
        if (getGroundDrag) {
            drag_coef = getGroundDrag(data.pos.x(), data.pos.z());
        }

        if (drag_coef > 0.0f) {
            totalForceWorld -= data.vel * drag_coef;
        }

        float yaw_drag_coef = getGroundYawDrag ? getGroundYawDrag(data.pos.x(), data.pos.z()) : 0.0f;
        if (yaw_drag_coef > 0.0f) {
            totalTorqueLocal -= local_AngVel * yaw_drag_coef;
        }

        data.accel = totalForceWorld / cfg.mass;
        data.vel += data.accel * subDt;
        data.pos += data.vel * subDt;

        // === ВРАЩЕНИЕ ===
        Eigen::Vector3f gyro = local_AngVel.cross(I * local_AngVel);
        Eigen::Vector3f local_angAccel = invI * (totalTorqueLocal - gyro);

        // Интегрируем угловую скорость в локальной СК
        local_AngVel += local_angAccel * subDt;
        data.angVel = local_AngVel;  // остаётся в локальной

        // Обновление кватерниона через локальную угловую скорость
        float theta = local_AngVel.norm() * subDt * 0.5f;
        Eigen::Quaternionf dq;
        if (theta > 1e-6f) {
            Eigen::Vector3f axis = local_AngVel.normalized();
            dq.w() = std::cos(theta);
            dq.vec() = axis * std::sin(theta);
        } else {
            dq = Eigen::Quaternionf::Identity();
        }
        data.ori = (data.ori * dq).normalized();

        // Коррекция проваливания (без изменений)
        bool any_wheel_under_ground = false;
        float max_lift_needed = 0.0f;


        for(int i = 0; i < 4; ++i) {
                 Eigen::Vector3f world_r = data.ori * cfg.wheelOffsets[i];
                 Eigen::Vector3f p_mount = data.pos + world_r;
                 Eigen::Vector3f carDown = data.ori * -Eigen::Vector3f::UnitY();

                 Eigen::Vector3f p_wheel = p_mount + (carDown * data.wheelPos[i]);

                 float h_g = getGroundHeight ? getGroundHeight(p_wheel.x(), p_wheel.z()) : 0.0f;

                 if (p_wheel.y() < h_g - DEAD_ZONE) {

                     // Нижняя точка колеса (с учётом радиуса)
                     Eigen::Vector3f p_wheel_bottom = p_mount + carDown * (data.wheelPos[i] + cfg.wheel_radius);

                     float h_g = getGroundHeight ? getGroundHeight(p_wheel_bottom.x(), p_wheel_bottom.z()) : 0.0f;

                     float penetration = (h_g - DEAD_ZONE) - p_wheel_bottom.y();
                     max_lift_needed = std::max(max_lift_needed, penetration);
                     any_wheel_under_ground = true;
                 }
             }

        if (any_wheel_under_ground) {
            data.pos.y() += max_lift_needed;

            // Синхронизируем сжатие подвески
            for(int i = 0; i < 4; ++i) {
                Eigen::Vector3f world_r = data.ori * cfg.wheelOffsets[i];
                Eigen::Vector3f p_mount = data.pos + world_r;
                Eigen::Vector3f carDown = data.ori * -Eigen::Vector3f::UnitY();
                Eigen::Vector3f p_wheel_bottom = p_mount + carDown * (data.wheelPos[i] + cfg.wheel_radius);

                float h_g = getGroundHeight ? getGroundHeight(p_wheel_bottom.x(), p_wheel_bottom.z()) : 0.0f;
                float new_dist = p_mount.y() - h_g;  // от точки крепления

                if (new_dist < cfg.susp_length) {
                    data.wheelPos[i] = cfg.susp_length - new_dist;
                }
            }
        }
    }

//    m_debugTimer += dt;
//    if (m_debugTimer >= m_debugInterval) {
//        m_debugTimer = 0;

//        Eigen::Vector3f local_vel = data.ori.conjugate() * data.vel;
//        float v_fwd    = local_vel.z();
//        float yaw_rate = data.angVel.y();

//        qDebug() << "\n========== DEBUG ==========";
//        qDebug() << "Pos:" << data.pos.x() << data.pos.y() << data.pos.z();
//        qDebug() << "V_fwd:" << v_fwd << "m/s | YawRate:" << yaw_rate << "rad/s ("
//                 << yaw_rate * RADTODEG << "deg/s)";
//        qDebug() << "SteerAngle:" << data.steerAngle * RADTODEG << "deg"
//                 << "| Wheels:" << data.wheelAngularVel[0] << data.wheelAngularVel[1]
//                 << data.wheelAngularVel[2] << data.wheelAngularVel[3];
//        qDebug() << "Susp:" << data.wheelPos[0] << data.wheelPos[1]
//                 << data.wheelPos[2] << data.wheelPos[3];
//        qDebug() << "--- INPUT ---";
//        qDebug() << "  Throttle:" << ctrl.throttle << "| Brake:" << ctrl.brake
//                 << "| Steer:" << ctrl.steerAngle * RADTODEG << "deg";
//        qDebug() << "===========================\n";
//    }

//    m_debugTimer += dt;
//    if (m_debugTimer >= m_debugInterval) {
//        m_debugTimer = 0;

//        Eigen::Matrix3f rotMat = data.ori.toRotationMatrix();
//        Eigen::Matrix3f invRotMat = rotMat.transpose();
//        Eigen::Vector3f local_V = invRotMat * data.vel;

//        // Расчет углов Эйлера
//        Eigen::Vector3f euler = rotMat.eulerAngles(0, 1, 2);
//        float roll  = euler[0] * RADTODEG;
//        float pitch = euler[1] * RADTODEG;
//        float yaw   = euler[2] * RADTODEG;

//        auto normalizeAngle = [](float a) {
//            while (a > 180.0f) a -= 360.0f;
//            while (a < -180.0f) a += 360.0f;
//            return a;
//        };
//        roll = normalizeAngle(roll);
//        pitch = normalizeAngle(pitch);
//        yaw = normalizeAngle(yaw);

//        Eigen::Vector3f carUp = rotMat * Eigen::Vector3f::UnitY();
//        bool isUpsideDown = carUp.y() < 0;

//        qDebug() << "\n========== PHYSICS DEBUG ==========";
//        qDebug() << "--- POSITION & ORIENTATION ---";
//        qDebug() << "  World pos: ["
//                 << QString::number(data.pos.x(), 'f', 6) << ", "
//                 << QString::number(data.pos.y(), 'f', 6) << ", "
//                 << QString::number(data.pos.z(), 'f', 6) << "]";
//        qDebug() << "  Rotation: roll=" << roll << "° pitch=" << pitch << "° yaw=" << yaw << "°";

//        qDebug() << "\n--- VELOCITY ---";
//        qDebug() << "  Linear:  " << data.vel.norm() * 3.6f << "km/h"
//                 << " vector=[" << data.vel.x() << ", " << data.vel.y() << ", " << data.vel.z() << "] m/s";
//        qDebug() << "  Angular: " << data.angVel.norm() * RADTODEG << "°/s"
//                 << " vector=[" << data.angVel.x() << ", " << data.angVel.y() << ", " << data.angVel.z() << "] rad/s";

//        qDebug() << "\n--- GROUND CONTACT ---";
//        float groundH_center = (getGroundHeight) ? getGroundHeight(data.pos.x(), data.pos.z()) : 0.0f;
//        qDebug() << "  Ground under center:" << groundH_center << "m";
//        qDebug() << "  Clearance (center):" << (data.pos.y() - groundH_center) << "m";
//        qDebug() << "  Upside down:" << (isUpsideDown ? "YES" : "NO");

//        qDebug() << "\n--- WHEELS DETAIL ---";
//        qDebug() << "Wheel | world_pos_y | ground_h | dist | contact | f_z(N) | v_rel";
//        qDebug() << "------|-------------|----------|------|---------|--------|-------";

//        const char* names[] = {"FL", "FR", "RL", "RR"};
//        for (int i = 0; i < 4; ++i) {
//            const auto& offset = cfg.wheelOffsets[i];

//            // Точка крепления (как в основном цикле)
//            Eigen::Vector3f world_r = rotMat * offset;
//            Eigen::Vector3f p_wheel = data.pos + world_r;

//            // Скорость в локальной СК
//            Eigen::Vector3f local_v_mount = local_V + data.angVel.cross(offset);
//            Eigen::Vector3f v_wheel = rotMat * local_v_mount;

//            float h_pixel = (getGroundHeight) ? getGroundHeight(p_wheel.x(), p_wheel.z()) : 0.0f;
//            Eigen::Vector3f groundNormal = (getGroundNormal) ? getGroundNormal(p_wheel.x(), p_wheel.z()) : Eigen::Vector3f::UnitY();

//            float dist = p_wheel.y() - h_pixel;
//            bool contact = (dist < cfg.susp_length && carUp.dot(groundNormal) > 0.1f);

//            float f_z = 0.0f;
//            float v_rel = 0.0f;
//            if (contact) {
//                float compression = cfg.susp_length - dist;
//                v_rel = v_wheel.dot(groundNormal);
//                float damping_coeff = (v_rel >= 0.0f) ? cfg.damping_compression : cfg.damping_rebound;
//                // ТАК ЖЕ КАК В ОСНОВНОМ ЦИКЛЕ — ПРОГРЕССИВНАЯ ПРУЖИНА
//                float F_susp_raw = compression * cfg.k_spring * (1.0f + compression * 5.0f) - v_rel * damping_coeff;
//                const float MAX_F = 4.0f * cfg.mass * gravity_v;
//                f_z = MAX_F * std::tanh(std::max(0.0f, F_susp_raw) / MAX_F);
//            }

//            qDebug().nospace().noquote()
//                << QString("%1   | %2 | %3 | %4 | %5 | %6 | %7")
//                    .arg(names[i])
//                    .arg(p_wheel.y(), 9, 'f', 3)
//                    .arg(h_pixel, 8, 'f', 3)
//                    .arg(dist, 4, 'f', 3)
//                    .arg(contact ? "YES   " : "NO    ")
//                    .arg(f_z, 6, 'f', 0)
//                    .arg(v_rel, 5, 'f', 2);
//        }

//        qDebug() << "\n--- WHEEL POSITIONS (state) ---";
//        qDebug() << "  wheelPos: [" << data.wheelPos[0] << ", " << data.wheelPos[1] << ", "
//                 << data.wheelPos[2] << ", " << data.wheelPos[3] << "]";

//        qDebug() << "=========================================\n";
//    }

    return data;
}

float Simulation_Car::gaussianNoise(float mean, float stddev) {
    if (stddev <= 0.0f) return mean;

    static float spare;
    static bool hasSpare = false;

    if (hasSpare) {
        hasSpare = false;
        return mean + stddev * spare;
    }

    hasSpare = true;
    float u, v, s;
    do {
        u = (rand() / (float)RAND_MAX) * 2.0f - 1.0f;
        v = (rand() / (float)RAND_MAX) * 2.0f - 1.0f;
        s = u * u + v * v;
    } while (s >= 1.0f || s == 0.0f);

    s = sqrtf(-2.0f * logf(s) / s);
    spare = v * s;
    return mean + stddev * u * s;
}

state_vec Simulation_Car::getNoisyState(float dt) {
    state_vec noisy = data;

    // ===== 1. Зашумлённый гироскоп (bias RW + instability + white) =====

    // 1a. Bias Random Walk — медленный уход нуля
    noise_state.gyro_bias.x() += gaussianNoise(0.0f, noise.gyro_rw_std * sqrtf(dt));
    noise_state.gyro_bias.y() += gaussianNoise(0.0f, noise.gyro_rw_std * sqrtf(dt));
    noise_state.gyro_bias.z() += gaussianNoise(0.0f, noise.gyro_rw_std * sqrtf(dt));

    // 1b. Bias Instability — марковский процесс (медленные флуктуации)
    float alpha = expf(-dt / 100.0f);  // постоянная времени 100с
    noise_state.gyro_bias_inst = alpha * noise_state.gyro_bias_inst +
        (1.0f - alpha) * Eigen::Vector3f(
            gaussianNoise(0.0f, noise.gyro_bias_inst_std),
            gaussianNoise(0.0f, noise.gyro_bias_inst_std),
            gaussianNoise(0.0f, noise.gyro_bias_inst_std));

    // 1c. Суммируем все шумы
    noisy.angVel.x() += gaussianNoise(0.0f, noise.gyro_white_std)
                      + noise_state.gyro_bias.x()
                      + noise_state.gyro_bias_inst.x();
    noisy.angVel.y() += gaussianNoise(0.0f, noise.gyro_white_std)
                      + noise_state.gyro_bias.y()
                      + noise_state.gyro_bias_inst.y();
    noisy.angVel.z() += gaussianNoise(0.0f, noise.gyro_white_std)
                      + noise_state.gyro_bias.z()
                      + noise_state.gyro_bias_inst.z();

    // ===== 2. Зашумлённый кватернион =====
    Eigen::Vector3f noise_axis = Eigen::Vector3f::Random().normalized();
    float noise_angle = gaussianNoise(0.0f, noise.attitude_white_std);
    Eigen::Quaternionf noise_quat(Eigen::AngleAxisf(noise_angle, noise_axis));
    noisy.ori = (data.ori * noise_quat).normalized();

    // ===== 3. Зашумлённая скорость (энкодер) =====
    Eigen::Vector3f local_vel = data.ori.conjugate() * data.vel;
    local_vel.z() += gaussianNoise(0.0f, noise.encoder_white_std);
    noisy.vel = data.ori * local_vel;

    return noisy;
}


float Simulation_Car::pacejka_with_params(float x, const PacejkaParams& p) {
    float bx = p.B * x;
    return p.D * std::sin(p.C * std::atan(bx - p.E * (bx - std::atan(bx))));
}

float Simulation_Car::getMotorTorque(float wheelOmega, float throttleCmd) const
{
    // Обороты мотора из оборотов колеса
    float motorRPM = std::abs(wheelOmega) * cfg.gear_ratio * 60.0f / (2.0f * M_PI);
    motorRPM = std::clamp(motorRPM, 0.0f, cfg.motor_max_rpm);

    // Кривая момента электромотора
    float torque;
    if (motorRPM < cfg.motor_peak_rpm) {
        // Зона постоянного момента
        torque = cfg.motor_max_torque;
    } else {
        // Зона постоянной мощности (момент падает)
        float omega_rad = motorRPM * 2.0f * M_PI / 60.0f;
        torque = cfg.motor_max_power / omega_rad;
        torque = std::min(torque, cfg.motor_max_torque);
    }

    // Направление момента
    //float direction = (throttleCmd >= 0.0f) ? 1.0f : -1.0f;

    return torque * throttleCmd;
}
