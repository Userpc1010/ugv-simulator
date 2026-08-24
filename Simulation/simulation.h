#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <QDebug>
#include "terrain_model.h"

#define pow2(a) ((a) * (a))
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define HALFPI 1.5707963267948966192313216916398f
#define PI 3.1415926535897932384626433832795f
#define TWOPI 6.283185307179586476925286766559f

const float WORLD_SCALE = 7.9869398f;//7,9869398207426376440460947503201
const float gravity_v = 9.81f;

struct control_vec {
    float steerAngle = 0.0f;   // целевой угол поворота руля (рад)
    float throttle = 0.0f;     // 0..1
    float brake = 0.0f;        // 0..1
};

struct sensor_noise {
    // Gyro
    float gyro_white_std       = 0.02f;   // рад/с — белый шум
    float gyro_rw_std          = 0.0001f; // рад/с — bias random walk
    float gyro_bias_inst_std   = 0.0005f; // рад/с — bias instability

    // Encoder
    float encoder_white_std    = 0.05f;   // м/с

    // Attitude
    float attitude_white_std   = 0.01f;  // рад
};

struct imu_noise_state {
    Eigen::Vector3f gyro_bias      = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_bias_inst = Eigen::Vector3f::Zero();
};

struct car_config {
    const float mass = 2200.0f;
    const Eigen::Vector3f inertia = {2500.0f, 3500.0f, 4000.0f};
    const float susp_length = 0.55f;               // Ход пружин подвески + камеры колёс

    // Пружины и демпферы
    float k_spring = 25000.0f;                     //Сила пружины
    float damping_compression  = 4000.0f;          //Затухание пружины
    float damping_rebound = 8000.0f;              // отбой

    // Рулевое
    const float max_sterr_rad = 0.523599f;          // Макс. угол (30 градусов)
    const float max_sterr_speed = 2.5f;             // Скорость (радианы в сек)

    // Стабилизаторы
    const float arb_stiffness_front = 100.0f;      // Anti roll barr предний
    const float arb_stiffness_rear = 100.0f;
    const float arb_max_force = 200.0f;

    // Дифференциал
    float diff_lock = 100.0f;                      // Коэффициент блокировки (Н·м)
    float center_diff_lock = 50.0f;                // блокировка межосевого дифференциала
    float rolling_resistance = 10.0f;              // сопротивление качению

    // Колёса и шины
    float wheel_radius = 0.35f;                     // радиус колеса
    const float wheel_mass = 60.0f;

    // Геометрия
    float wheelbase = 7.107f;                        //Колесная база
    float trackWidth = 3.675f;                       //Колея
    float ackermannFactor = 0.258f;                  //Коэффициент коррекции поворота колеса

    // Сцепление
    float mu = 2.0f;                                  // Сцепление колёс с землёй
    const float wheelWidth = 0.3f;                   // Ширина колеса

    // Двигатель и тормоза

    // Электродвигатель
    float motor_max_torque = 1250.0f;      // Н·м пиковый момент на валу
    float motor_max_power = 250000.0f;    // 180 кВт максимальная мощность
    float motor_max_rpm = 12000.0f;       // максимальные обороты
    float motor_peak_rpm = 3000.0f;       // обороты до которых момент постоянный
    float gear_ratio = 12.0f;             // редуктор мотор:колесо
    float drivetrain_efficiency = 0.92f;  // КПД редуктора

    const float brake = 25000.0f;                     //Тормоз

    // Pacejka 
    PacejkaParams pacejka_default;

    // Колёсные offsets вычисляем позицию стакана как центр колеса в арке -0.9987
    // (ниже ЦМ) + половина хода пружины т.к. обычно пружина загружена на половину
    std::vector<Eigen::Vector3f> wheelOffsets = {
        { -1.837624f, -0.9987f + susp_length * 0.5f, 3.292974f },
        {  1.837624f, -0.9987f + susp_length * 0.5f, 3.292974f },
        { -1.837624f, -0.9987f + susp_length * 0.5f, -3.81111f },
        {  1.837624f, -0.9987f + susp_length * 0.5f, -3.81111f }
    };
};

struct state_vec {
    Eigen::Vector3f pos = {0, 1.0f, 0};
    Eigen::Vector3f vel = {0, 0, 0};
    Eigen::Vector3f accel = {0, 0, 0};
    Eigen::Quaternionf ori = Eigen::Quaternionf::Identity();
    Eigen::Vector3f angVel = {0, 0, 0};
    float steerAngle = 0.0f;
    Eigen::Vector4f wheelPos = {0.25f, 0.25f, 0.25f, 0.25f}; // Вертикальное смещение (м)
    Eigen::Vector4f wheelVel = Eigen::Vector4f::Zero(); // Вертикальная скорость (м/с)
    Eigen::Vector4f wheelAngularVel = Eigen::Vector4f::Zero(); // рад/с
    Eigen::Vector4f wheelRotation = Eigen::Vector4f::Zero();   // накопленный угол (рад)
};

class Simulation_Car {
public:
    Simulation_Car();
    state_vec update(const control_vec& ctrl, float dt);
    void updateDampingSpring();
    void reset();

    sensor_noise noise;

    state_vec getNoisyState(float dt);

    std::function<float(float, float)> getGroundHeight = nullptr;
    std::function<Eigen::Vector3f(float, float)> getGroundNormal = nullptr;

    std::function<float(float, float, const Eigen::Quaternionf&)> getGroundMu = nullptr;
    std::function<float(float, float, const Eigen::Quaternionf&)> getGroundRR = nullptr;

    std::function<float(float, float)> getGroundDrag = nullptr;
    std::function<float(float, float)> getGroundYawDrag = nullptr;

    std::function<PacejkaParams(float, float)> getGroundPacejka = nullptr;

    car_config cfg;

private:

    float gaussianNoise(float mean, float stddev);

    float pacejka_with_params(float x, const PacejkaParams& p);

    float getMotorTorque(float wheelOmega, float throttleCmd) const;

    state_vec data;

    imu_noise_state noise_state;

    float m_debugTimer = 0.0f;

    const float m_debugInterval = 1.0; // 100 мс

    const float DEAD_ZONE = 0.2f;

};
#endif
