#include "oglwidget.h"
#include "simpleobject3d.h"
#include "transformational.h"
#include "camera_3d.h"
#include "cube.h"
#include "skybox.h"
#include "statelattice.h"
#include "mppi-generic.h"
#include "rpp_controller.hpp"
#include "vector_pursuit_controller.hpp"
#include <QCoreApplication>
#include <QQuaternion>
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QtMath>
#include <QDir>

OGLWidget::OGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_cameraMode(CameraMode_Follow)
    , m_cameraOrbitYaw(0.0)
    , m_cameraOrbitPitch(25.0)
    , m_cameraDistance(15.0)
    , m_camForward(false)
    , m_camBackward(false)
    , m_camLeft(false)
    , m_camRight(false)
    , m_camUp(false)
    , m_camDown(false)
    , m_ProjectionMatrix()
{
    m_camera = new Camera_3D;
    m_cube = new Cube;

    qRegisterMetaType<Path>("Path");
    qRegisterMetaType<uint16_t>("uint16_t");
    qRegisterMetaType<std::vector<std::tuple<float, float, float>>>("std::vector<std::tuple<float, float, float>>");
}

OGLWidget::~OGLWidget()
{
 delete m_camera;
 delete m_stateLattice;
 delete m_controller;
 delete m_terrain;
}

void OGLWidget::closeEvent(QCloseEvent *event) {
    makeCurrent();

    delete m_SkyBox;
    m_SkyBox = nullptr;
    delete m_cube;
    m_cube = nullptr;
    delete m_costmapRander;
    m_costmapRander = nullptr;
    delete m_MultiLineRender;
    m_MultiLineRender = nullptr;

    // Очищаем объекты
    for(auto o: m_Objects) delete o;
    m_Objects.clear();
    for(auto o: m_OBJ_Objects) delete o;
    m_OBJ_Objects.clear();

    delete Truck;
    Truck = nullptr;
    delete Truck_wheel_back_right;
    delete Truck_wheel_back_left;
    delete Truck_wheel_front_right;
    delete Truck_wheel_front_left;

    delete Terrain;
    Terrain = nullptr;
    delete m_GoalTruck;
    m_GoalTruck = nullptr;
    delete m_GoalWheelFR;
    delete m_GoalWheelFL;
    delete m_GoalWheelRR;
    delete m_GoalWheelRL;

    delete m_pathLines;
    delete m_expLines;
    delete m_mppiLines;
    delete m_MultiLineRender;
    delete m_perimeterLines;

    doneCurrent();
    event->accept();
}

void OGLWidget::initializeGL()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    initShaders();

    QtImGui::initialize(this);

    QString appPath = QCoreApplication::applicationDirPath();

    Truck = new SimpleObject3D(":/Pickup/Truck_hull.obj", QImage(":/Pickup/pickup_diffuse.png"));
    Truck_wheel_back_right = new SimpleObject3D(":/Pickup/Truck_wheel_back_right.obj", QImage(":/Pickup/wheels2.png"));
    Truck_wheel_back_left = new SimpleObject3D(":/Pickup/Truck_wheel_back_left.obj", QImage(":/Pickup/wheels2.png"));
    Truck_wheel_front_right = new SimpleObject3D(":/Pickup/Truck_wheel_front_right.obj", QImage(":/Pickup/wheels2.png"));
    Truck_wheel_front_left = new SimpleObject3D(":/Pickup/Truck_wheel_front_left.obj", QImage(":/Pickup/wheels2.png"));

    // Создаём копию модели для цели
    m_GoalTruck = new SimpleObject3D(":/Pickup/Truck_hull.obj", QImage(":/Pickup/pickup_diffuse.png"));
    m_GoalWheelFR = new SimpleObject3D(":/Pickup/Truck_wheel_front_right.obj", QImage(":/Pickup/wheels2.png"));
    m_GoalWheelFL = new SimpleObject3D(":/Pickup/Truck_wheel_front_left.obj", QImage(":/Pickup/wheels2.png"));
    m_GoalWheelRR = new SimpleObject3D(":/Pickup/Truck_wheel_back_right.obj", QImage(":/Pickup/wheels2.png"));
    m_GoalWheelRL = new SimpleObject3D(":/Pickup/Truck_wheel_back_left.obj", QImage(":/Pickup/wheels2.png"));

    Terrain = new SimpleObject3D(appPath + "/Terrain/untitled.obj", QImage(appPath + "/Terrain/Textures7.png"));
    Terrain->rotate(QQuaternion::fromEulerAngles(0.0f, 0.0f, 0.0f));
    Terrain->translate(QVector3D(0.0f, (-41.695f / WORLD_SCALE) / WORLD_SCALE, 0.0f));
    Terrain->scale(1.0f / WORLD_SCALE);

    m_terrain = new TerrainModel(appPath + "/Terrain/output.r16", 4097, 4097,
                                  16000.0f / WORLD_SCALE, 16000.0f / WORLD_SCALE,
                                  2625.0f / WORLD_SCALE);

    Sim.getGroundHeight = [this](float x, float z) {
        return m_terrain->getHeight(x, z);
    };
    Sim.getGroundNormal = [this](float x, float z) {
        return m_terrain->getNormal(x, z);
    };

    Sim.getGroundMu = [this](float x, float z, const Eigen::Quaternionf& ori) {
        return m_terrain->getZoneMu(x, z, ori);
    };

    Sim.getGroundRR = [this](float x, float z, const Eigen::Quaternionf& ori) {
        return m_terrain->getZoneRollingResistance(x, z, ori);
    };

    Sim.getGroundDrag = [this](float x, float z) {
        return m_terrain->getZoneDrag(x, z);
    };

    Sim.getGroundYawDrag = [this](float x, float z) {
        return m_terrain->getZoneYawDrag(x, z);
    };

    Sim.getGroundPacejka = [this](float x, float z) -> PacejkaParams {
        auto p = m_terrain->getZonePacejka(x, z);
        return {p.B, p.C, p.D, p.E};
    };

    // Инициализация costmap
    m_terrain->computeSlopeMap(2);  // SA=2
    //m_terrain->dumpHeightSample();

    m_terrain->loadSurfaceZones(
        appPath + "/Terrain/Earth.bmp",
        appPath + "/Terrain/Forest.bmp",
        appPath + "/Terrain/Mountain_road.bmp",
        appPath + "/Terrain/Passages.bmp",
        appPath + "/Terrain/Swamp.bmp"
    );

    m_costmapRander = new CostmapRander();
    m_costmapRander->init(16000.0f / WORLD_SCALE, 16000.0f / WORLD_SCALE, 0.0f);
    m_costmapRander->update_map(m_terrain->getCostmapImage());

    m_MultiLineRender = new MultiDrawLine();
    m_pathLines = new MultiDrawLine();
    m_expLines = new MultiDrawLine();
    m_mppiLines = new MultiDrawLine();

    m_perimeterLines = new MultiDrawLine();

    // Создаём и настраиваем StateLattice
    m_stateLattice = new StateLattice(this);

    // Подключаем сигналы визуализации от StateLattice
    connect(m_stateLattice, SIGNAL(DrawLines(GLfloat*, QVector3D, unsigned long long, uint16_t)),
            this, SLOT(onDrawLines(GLfloat*, QVector3D, unsigned long long, uint16_t)));

    connect(m_stateLattice, SIGNAL(planningFinished(bool, const Path&,
                                                    const std::vector<std::tuple<float, float, float>>&,
                                                    double)),
            this, SLOT(onPlanningFinished(bool, const Path&,
                                          const std::vector<std::tuple<float, float, float>>&,
                                          double)));

    // Инициализация MPPI
    m_mppiController = new MPPIController(this);

    float costmap_res = m_terrain->getWidth() / m_terrain->getCostmapWidth();
    float costmap_origin_x = -m_terrain->getWidth() / 2.0f;
    float costmap_origin_y = -m_terrain->getHeight() / 2.0f;

    bool mppi_ok = m_mppiController->initialize(
        0.1f, 1.0f, 20.0f,           // dt, lambda, desired_speed
        800, 800,                      // costmap width, height
        1.0f,                           // resolution
        0, 0
    );

    if (mppi_ok) {
        m_mppiInitialized = true;
        m_mppiCostmapBuffer.resize(800 * 800);
        qDebug() << "[OGLWidget] MPPI Controller initialized";
    } else {
        qWarning() << "[OGLWidget] MPPI Controller initialization failed";
    }

    m_rppController = new rpp::RPPController();

        rpp::RPPParams rpp_params;

        m_rppController->configure(
               rpp_params,
               m_terrain->getCostmapData(),
               m_terrain->getCostmapWidth(),
               m_terrain->getCostmapHeight(),
               1.0f,   // разрешение такое же как у MPPI
               0.0f,   // origin_x — будет обновлено через updateCostmap
               0.0f    // origin_y
           );

        qDebug() << "[OGLWidget] RPP Controller initialized";

        m_lastPlannedGoalPosition = QVector3D(0, 0, 0);
        m_pathNeedsReplan = true;

      // Инициализация Vector Pursuit Controller
      m_vpcController = new vpc::VectorPursuitController();

        vpc::VPCParams vpc_params;

        m_vpcController->configure(vpc_params);

    qDebug() << "[OGLWidget] VPC Controller initialized";

    m_controller = new CarController();
    m_controller->max_steer      = Sim.cfg.max_sterr_rad;
    m_controller->wheelbase      = Sim.cfg.wheelbase;
    m_controller->max_centrifugal = 4.0f;

    m_SkyBox = new SkyBox(4000.0f,
                          QImage(":/sky/sky_forward.png"),
                          QImage(":/sky/sky_top.png"),
                          QImage(":/sky/Terrain_3.jpg"),
                          QImage(":/sky/sky_left.png"),
                          QImage(":/sky/sky_right.png"),
                          QImage(":/sky/sky_back.png"));

    if(!sim_timer.isActive()) sim_timer.start(10, this);
}

void OGLWidget::timerEvent(QTimerEvent *event) {
    Q_UNUSED(event);

    if (m_cameraMode == CameraMode_Follow) {
        if (keyW) m_v_setpoint += v_rate * FIXED_DT;
        if (keyS) m_v_setpoint -= v_rate * FIXED_DT;
        m_v_setpoint = std::clamp(m_v_setpoint, v_min, v_max);

        if (keyA) m_w_setpoint += w_rate * FIXED_DT;
        if (keyD) m_w_setpoint -= w_rate * FIXED_DT;
        m_w_setpoint = std::clamp(m_w_setpoint, -w_max, w_max);

        if (keyShift) {
            m_w_setpoint = 0.0f;
        }
        if (keySpace) {
            m_v_setpoint = 0.0f;
            m_w_setpoint = 0.0f;
        }
    }

    state_vec noisy = Sim.getNoisyState(FIXED_DT);
    m_controller->setCommands(m_w_setpoint, m_v_setpoint);
    control_vec ctrl = m_controller->update(noisy, FIXED_DT);
    data = Sim.update(ctrl, FIXED_DT);

    m_noisy_yaw_rate = noisy.angVel.y();
    m_noisy_local_vel = noisy.ori.conjugate() * noisy.vel;

    mppi_conunter++;

    // --- MPPI CONTROL ---
    if (m_controllerMode == ControllerMode_MPPI && m_mppiInitialized && mppi_conunter >= 10) {

        mppi_conunter = 0;

        // Вычисляем forward и local_vel для MPPI
        Eigen::Vector3f forward = data.ori * Eigen::Vector3f::UnitZ();
        Eigen::Vector3f local_vel = data.ori.conjugate() * data.vel;
        m_noisy_yaw_rate = noisy.angVel.y();
        m_noisy_local_vel = local_vel;

        // 1. Обновить costmap (локальное окно вокруг робота)
        float cellSize = m_terrain->getWidth() / m_terrain->getCostmapWidth();
        float halfW = m_terrain->getWidth() / 2.0f;
        float halfH = m_terrain->getHeight() / 2.0f;

        int centerX = static_cast<int>((data.pos.x() + halfW) / cellSize);
        int centerZ = static_cast<int>((data.pos.z() + halfH) / cellSize);

        m_mppiCostmapBuffer = extractLocalCostmap(centerX, centerZ, 800);
        m_mppiController->setCostmapCPU(m_mppiCostmapBuffer.data());

        // 2. Позиция робота в ЛОКАЛЬНЫХ координатах costmap (всегда центр)
        float robot_cx = 400.0f;
        float robot_cz = 400.0f;

        // 3. Цель в локальных координатах costmap
        int goalX_idx = static_cast<int>((m_GoalPosition.x() + halfW) / cellSize);
        int goalZ_idx = static_cast<int>((m_GoalPosition.z() + halfH) / cellSize);
        goalX_idx = std::clamp(goalX_idx, 0, m_terrain->getCostmapWidth() - 1);
        goalZ_idx = std::clamp(goalZ_idx, 0, m_terrain->getCostmapHeight() - 1);

        float goal_cx = 400.0f + (goalX_idx - centerX);
        float goal_cz = 400.0f + (goalZ_idx - centerZ);

        // Yaw цели в радианах

        float goalYaw = qDegreesToRadians(m_GoalRotation.toEulerAngles().y()) + M_PI/2;
        m_mppiController->setGoal(goal_cx, goal_cz, goalYaw);


        // 4. Получить управление (координаты в ЛОКАЛЬНОЙ СК costmap!)
        MPPI_Output mppi_out = m_mppiController->computeControlSync(
            robot_cx, robot_cz,
            std::atan2(forward.z(), forward.x()),
            local_vel.z(),
            -data.steerAngle
        );

        static int mppi_call = 0;
        if (mppi_call++ % 10 == 0) {

            float psi = std::atan2(forward.z(), forward.x()) * RADTODEG;
            float goal_dist = std::hypot(data.pos.x() - m_GoalPosition.x(), data.pos.z() - m_GoalPosition.z());

            // Получаем траекторию состояний
            std::vector<float> state_traj = m_mppiController->getStateTrajectory();
            int timesteps = state_traj.size() / 3;

            // Координаты X,Y и курс для каждого 10-го шага + последний
            QString traj;
            for (int t = 0; t < timesteps; t += 10) {
                float x = state_traj[t*3 + 0];
                float y = state_traj[t*3 + 1];
                float psi_val = state_traj[t*3 + 2] * RADTODEG;
                traj += QString("t%1:(%2,%3 %4°)|").arg(t).arg(x, 0, 'f', 1).arg(y, 0, 'f', 1).arg(psi_val, 0, 'f', 1);
            }
            // Последний шаг
            int last = timesteps - 1;
            traj += QString("END:(%1,%2 %3°)").arg(state_traj[last*3], 0, 'f', 1).arg(state_traj[last*3+1], 0, 'f', 1).arg(state_traj[last*3+2] * RADTODEG, 0, 'f', 1);

            qDebug() << "[MPPI]"
                     << "IN: pos=" << robot_cx << "," << robot_cz
                     << "psi=" << psi << "°"
                     << "v=" << local_vel.z() << "m/s"
                     << "steer=" << data.steerAngle * RADTODEG << "°"
                     << "GOAL:" << goal_cx << "," << goal_cz << "yaw=" << goalYaw * RADTODEG << "°"
                     << "dist=" << goal_dist << "m"
                     << "OUT: v_cmd=" << mppi_out.control_v << " w_cmd=" << mppi_out.control_w
                     << "cost=" << mppi_out.cost
                     << "TRAJ:" << traj;
        }

        // 5. Установить команды для CarController
        m_v_setpoint = mppi_out.control_v;
        m_w_setpoint = -mppi_out.control_w;
    }

    // --- RPP CONTROL ---
    if (m_controllerMode == ControllerMode_RPP && m_rppController && mppi_conunter >= 10) {
        mppi_conunter = 0;

        float cellSize = m_terrain->getWidth() / m_terrain->getCostmapWidth();
        float halfW = m_terrain->getWidth() / 2.0f;
        float halfH = m_terrain->getHeight() / 2.0f;

        int centerX = static_cast<int>((data.pos.x() + halfW) / cellSize);
        int centerZ = static_cast<int>((data.pos.z() + halfH) / cellSize);

        m_mppiCostmapBuffer = extractLocalCostmap(centerX, centerZ, 800);

        // Costmap обновляем всегда
        m_rppController->updateCostmap(
            m_mppiCostmapBuffer.data(),
            800, 800,
            cellSize,
            (centerX - 400) * cellSize - halfW,
            (centerZ - 400) * cellSize - halfH
        );

        // ✅ Перепланируем ТОЛЬКО если нужно
        float goal_moved = std::hypot(m_GoalPosition.x() - m_lastPlannedGoalPosition.x(),
                                       m_GoalPosition.z() - m_lastPlannedGoalPosition.z());
        if (goal_moved > 0.5f) {
            m_pathNeedsReplan = true;
            m_lastPlannedGoalPosition = m_GoalPosition;
        }

        if (m_pathNeedsReplan && !m_stateLattice->isPlanningInProgress() && m_GoalPosition != QVector3D()) {
            m_stateLattice->updateCostmapFromData(m_mppiCostmapBuffer.data());
            m_stateLattice->localOffsetX = (centerX - 400) * cellSize - halfW;
            m_stateLattice->localOffsetY = (centerZ - 400) * cellSize - halfH;

            int goalX_idx = static_cast<int>((m_GoalPosition.x() + halfW) / cellSize);
            int goalZ_idx = static_cast<int>((m_GoalPosition.z() + halfH) / cellSize);
            goalX_idx = std::clamp(goalX_idx, 0, m_terrain->getCostmapWidth() - 1);
            goalZ_idx = std::clamp(goalZ_idx, 0, m_terrain->getCostmapHeight() - 1);

            int localGoalX = 400 + (goalX_idx - centerX);
            int localGoalZ = 400 + (goalZ_idx - centerZ);

            Eigen::Vector3f forward = data.ori * Eigen::Vector3f::UnitZ();
            float startYaw = std::atan2(forward.x(), forward.z());
            float goalYaw = qDegreesToRadians(m_GoalRotation.toEulerAngles().y());

            Pose start, goal;
            start.position = Eigen::Vector2f(400.0f, 400.0f);
            start.orientation = -startYaw + M_PI/2;
            goal.position = Eigen::Vector2f(localGoalX, localGoalZ);
            goal.orientation = -goalYaw + M_PI/2;

            m_stateLattice->asyncGetPathToTarget(start, goal);
        }

        // Вызов контроллера
        float robot_world_x = data.pos.x();
        float robot_world_z = data.pos.z();

        Eigen::Vector3f forward = data.ori * Eigen::Vector3f::UnitZ();
        float carYaw = -std::atan2(forward.x(), forward.z()) + M_PI/2;
        Eigen::Vector3f local_vel = data.ori.conjugate() * data.vel;

        rpp::RPPOutput rpp_out = m_rppController->computeVelocityCommands(
            robot_world_x, robot_world_z, carYaw,
            local_vel.z(), data.angVel.y(),
            0.1f
        );

        m_v_setpoint = rpp_out.linear_vel;
        m_w_setpoint = -rpp_out.angular_vel;

        static int rpp_log_counter = 0;
        if (rpp_log_counter++ % 10 == 0) {
            float lx, ly;
            m_rppController->getLookaheadPoint(lx, ly);
            qDebug() << "[RPP]"
                     << "pos:" << data.pos.x() << "," << data.pos.z()
                     << "v:" << local_vel.z() << "m/s"
                     << "OUT: v_cmd:" << rpp_out.linear_vel
                     << "w_cmd:" << rpp_out.angular_vel
                     << "lookahead:" << lx << "," << ly;
        }
    }

    // --- VECTOR PURSUIT CONTROL ---
    if (m_controllerMode == ControllerMode_VPC && m_vpcController && mppi_conunter >= 10) {
        mppi_conunter = 0;

        float cellSize = m_terrain->getWidth() / m_terrain->getCostmapWidth();
        float halfW = m_terrain->getWidth() / 2.0f;
        float halfH = m_terrain->getHeight() / 2.0f;

        int centerX = static_cast<int>((data.pos.x() + halfW) / cellSize);
        int centerZ = static_cast<int>((data.pos.z() + halfH) / cellSize);

        m_mppiCostmapBuffer = extractLocalCostmap(centerX, centerZ, 800);

        // Costmap обновляем всегда
        m_vpcController->updateCostmap(
            m_mppiCostmapBuffer.data(),
            800, 800,
            cellSize,
            (centerX - 400) * cellSize - halfW,
            (centerZ - 400) * cellSize - halfH
        );

        // ✅ Перепланируем ТОЛЬКО если нужно
        float goal_moved = std::hypot(m_GoalPosition.x() - m_lastPlannedGoalPosition.x(),
                                       m_GoalPosition.z() - m_lastPlannedGoalPosition.z());
        if (goal_moved > 0.5f) {
            m_pathNeedsReplan = true;
            m_lastPlannedGoalPosition = m_GoalPosition;
        }

        if (m_pathNeedsReplan && !m_stateLattice->isPlanningInProgress() && m_GoalPosition != QVector3D()) {
            m_stateLattice->updateCostmapFromData(m_mppiCostmapBuffer.data());
            m_stateLattice->localOffsetX = (centerX - 400) * cellSize - halfW;
            m_stateLattice->localOffsetY = (centerZ - 400) * cellSize - halfH;

            int goalX_idx = static_cast<int>((m_GoalPosition.x() + halfW) / cellSize);
            int goalZ_idx = static_cast<int>((m_GoalPosition.z() + halfH) / cellSize);
            goalX_idx = std::clamp(goalX_idx, 0, m_terrain->getCostmapWidth() - 1);
            goalZ_idx = std::clamp(goalZ_idx, 0, m_terrain->getCostmapHeight() - 1);

            int localGoalX = 400 + (goalX_idx - centerX);
            int localGoalZ = 400 + (goalZ_idx - centerZ);

            Eigen::Vector3f forward = data.ori * Eigen::Vector3f::UnitZ();
            float startYaw = std::atan2(forward.x(), forward.z());
            float goalYaw = qDegreesToRadians(m_GoalRotation.toEulerAngles().y());

            Pose start, goal;
            start.position = Eigen::Vector2f(400.0f, 400.0f);
            start.orientation = -startYaw + M_PI/2;
            goal.position = Eigen::Vector2f(localGoalX, localGoalZ);
            goal.orientation = -goalYaw + M_PI/2;

            m_stateLattice->asyncGetPathToTarget(start, goal);
        }

        // Вызов контроллера
        float robot_world_x = data.pos.x();
        float robot_world_z = data.pos.z();

        Eigen::Vector3f forward = data.ori * Eigen::Vector3f::UnitZ();
        float carYaw = -std::atan2(forward.x(), forward.z()) + M_PI/2;
        Eigen::Vector3f local_vel = data.ori.conjugate() * data.vel;

        vpc::VPCOutput vpc_out = m_vpcController->computeVelocityCommands(
            robot_world_x, robot_world_z, carYaw,
            local_vel.z(), data.angVel.y(),
            0.1f
        );

        m_v_setpoint = vpc_out.linear_vel;
        m_w_setpoint = -vpc_out.angular_vel;

        static int vpc_log_counter = 0;
        if (vpc_log_counter++ % 10 == 0) {
            float lx, ly;
            m_vpcController->getLookaheadPoint(lx, ly);

            qDebug() << "[VPC]"
                     << "pos:" << data.pos.x() << "," << data.pos.z()
                     << "v:" << local_vel.z() << "m/s"
                     << "OUT: v_cmd:" << vpc_out.linear_vel
                     << "w_cmd:" << vpc_out.angular_vel
                     << "steering:" << vpc_out.steering_angle * RADTODEG << "deg"
                     << "lookahead:" << lx << "," << ly
                     << "lookahead_dist:" << m_vpcController->getLookaheadDist()
                     << "lookahead_heading:" << m_vpcController->getLookaheadHeading() * RADTODEG << "deg"
                     << "turning_radius:" << m_vpcController->getTurningRadius()
                     << "curvature:" << m_vpcController->getCurvature()
                     << "sign:" << m_vpcController->getSign()
                     << "path_size:" << m_vpcController->getPathSize()
                     << "transformed_size:" << m_vpcController->getTransformedPathSize()
                     << "rotating:" << vpc_out.is_rotating_to_heading;
        }

        if (vpc_log_counter++ % 50 == 0) {  // реже, чтобы не засорять
            const auto& transformed = m_vpcController->getTransformedPath();
            qDebug() << "[VPC PATH] First 5 points (robot frame):";
            for (int i = 0; i < std::min(5, (int)transformed.poses.size()); ++i) {
                qDebug() << "  [" << i << "]"
                         << "x:" << transformed.poses[i].position.x()
                         << "y:" << transformed.poses[i].position.y()
                         << "heading:" << transformed.poses[i].orientation * RADTODEG << "deg";
            }
        }
    }

    update();
}

void OGLWidget::updateFollowCamera(const Eigen::Vector3f& carPos, const Eigen::Quaternionf& carOri)
{
    if (Truck == nullptr || m_terrain == nullptr) return;

    QVector3D qCarPos(carPos.x(), carPos.y(), carPos.z());
    QVector3D carVel = QVector3D(data.vel.x(), data.vel.y(), data.vel.z());

    QQuaternion qCarOri(carOri.w(), carOri.x(), carOri.y(), carOri.z());
    float carYaw = qCarOri.toEulerAngles().y();
    float totalYaw = carYaw + m_cameraOrbitYaw;

    QVector3D carForward = qCarOri.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f)).normalized();

    QVector3D flatForward = carForward;
    flatForward.setY(0);
    if (flatForward.isNull()) flatForward = QVector3D(0, 0, 1);
    flatForward.normalize();

    QQuaternion finalRot = QQuaternion::fromEulerAngles(m_cameraOrbitPitch, totalYaw, 0.0f);
    QVector3D cameraOffset = finalRot.rotatedVector(QVector3D(0.0f, 0.0f, -1.0f));

    QVector3D targetCamPos = qCarPos + (cameraOffset * m_cameraDistance) + QVector3D(0, 0.375f, 0);
    targetCamPos += carVel * 0.1f;

    if (m_firstFrame) {
        m_smoothedCamPos = targetCamPos;
        m_firstFrame = false;
    }

    float t = 0.2f;
    m_smoothedCamPos = m_smoothedCamPos * (1.0f - t) + targetCamPos * t;

    m_camera->setPosition(m_smoothedCamPos);
    QVector3D lookAtTarget = qCarPos + carForward * 0.375f + QVector3D(0.0f, 0.1875f, 0.0f);
    m_camera->lookAt(lookAtTarget);
}

void OGLWidget::resizeGL(int w, int h)
{
    float aspect = w / (h ? static_cast<float>(h) : 1);
    m_ProjectionMatrix.setToIdentity();
    m_ProjectionMatrix.perspective(88, aspect, 1.0f, 4001.0f);
}

void OGLWidget::paintGL()
{
    if (!initialized) {
        lastTime = std::chrono::steady_clock::now();
        initialized = true;
        m_prevData = data;
        return;
    }

    m_interpFactor = 0.9f;

    Eigen::Vector3f interpPos = m_prevData.pos * (1.0f - m_interpFactor) + data.pos * m_interpFactor;
    Eigen::Quaternionf interpOri = m_prevData.ori.slerp(m_interpFactor, data.ori);
    Eigen::Vector4f interpWheelRotation = m_prevData.wheelRotation * (1.0f - m_interpFactor) + data.wheelRotation * m_interpFactor;
    Eigen::Vector4f interpWheelPos = m_prevData.wheelPos * (1.0f - m_interpFactor) + data.wheelPos * m_interpFactor;
    float interpSteerAngle = m_prevData.steerAngle * (1.0f - m_interpFactor) + data.steerAngle * m_interpFactor;

    m_prevData = data;

    SimpleObject3D* wheels[] = { Truck_wheel_front_left, Truck_wheel_front_right,
                                 Truck_wheel_back_left, Truck_wheel_back_right };

    // Обновление камеры
    if (m_cameraMode == CameraMode_Follow) {
        updateFollowCamera(interpPos, interpOri);
    } else {
        if (m_camForward) m_camera->Front_move();
        if (m_camBackward) m_camera->Back_move();
        if (m_camLeft) m_camera->left_move();
        if (m_camRight) m_camera->right_move();
        if (m_camUp) m_camera->Up_move();
        if (m_camDown) m_camera->Down_move();
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ========== РЕЖИМ COSTMAP ==========
    if (m_showCostMap) {

        const float costmapCarY = 1.25f;

        // Камера
        if (m_cameraMode == CameraMode_Follow) {
            Eigen::Vector3f camPos = interpPos;
            camPos.y() = costmapCarY;
            updateFollowCamera(camPos, interpOri);
        } else {
            if (m_camForward) m_camera->Front_move();
            if (m_camBackward) m_camera->Back_move();
            if (m_camLeft) m_camera->left_move();
            if (m_camRight) m_camera->right_move();
            if (m_camUp) m_camera->Up_move();
            if (m_camDown) m_camera->Down_move();
        }

        m_Program.bind();
        m_Program.setUniformValue("u_projectionMatrix", m_ProjectionMatrix);
        m_Program.setUniformValue("u_map_offset", QVector3D(0, 0, 0));

        // Costmap
        m_costmapRander->draw(&m_Program, context()->functions());

        // Машина
        Eigen::Vector3f forward = interpOri * Eigen::Vector3f::UnitZ();
        forward.y() = 0.0f;
        if (forward.norm() < 1e-6f) forward = Eigen::Vector3f::UnitZ();
        forward.normalize();
        float carYaw = std::atan2(forward.x(), forward.z());
        Eigen::Quaternionf yawOnlyOri(Eigen::AngleAxisf(carYaw, Eigen::Vector3f::UnitY()));

        QQuaternion q_rot = QQuaternion::fromEulerAngles(0, carYaw * RADTODEG, 0);
        Truck->rotate_to(q_rot);
        Truck->translate_to(QVector3D(interpPos.x(), costmapCarY, interpPos.z()));
        Truck->draw(&m_Program, context()->functions());

        // Колёса машины
        for (int i = 0; i < 4; ++i) {
            if (!wheels[i]) continue;
            Eigen::Vector3f offset = Sim.cfg.wheelOffsets[i];
            Eigen::Vector3f rotatedOffset = yawOnlyOri * offset;
            QVector3D worldWheelPos(
                interpPos.x() + rotatedOffset.x(),
                costmapCarY + rotatedOffset.y(),
                interpPos.z() + rotatedOffset.z()
            );
            wheels[i]->translate_to(worldWheelPos);
            QQuaternion wheelRot = q_rot;
            if (i < 2) {
                wheelRot = wheelRot * QQuaternion::fromAxisAndAngle(0, 1, 0, interpSteerAngle * RADTODEG);
            }
            wheelRot = wheelRot * QQuaternion::fromAxisAndAngle(1, 0, 0, interpWheelRotation[i] * RADTODEG);
            wheels[i]->rotate_to(wheelRot);
            wheels[i]->draw(&m_Program, context()->functions());
        }

        // ======= Отрисовка цели ==============
        if (m_GoalPosition != QVector3D() && m_GoalTruck) {
            const float goalY = 1.25f;

            m_GoalTruck->rotate_to(m_GoalRotation);
            m_GoalTruck->translate_to(QVector3D(m_GoalPosition.x(), goalY, m_GoalPosition.z()));
            m_GoalTruck->draw(&m_Program, context()->functions());

            QQuaternion q(m_GoalRotation.scalar(), m_GoalRotation.x(),
                          m_GoalRotation.y(), m_GoalRotation.z());
            Eigen::Quaternionf eq(q.scalar(), q.x(), q.y(), q.z());

            SimpleObject3D* goalWheels[] = {m_GoalWheelFL, m_GoalWheelFR, m_GoalWheelRL, m_GoalWheelRR};

            for (int i = 0; i < 4; ++i) {
                Eigen::Vector3f offset = Sim.cfg.wheelOffsets[i];
                Eigen::Vector3f rotatedOffset = eq * offset;

                QVector3D wheelPos(m_GoalPosition.x() + rotatedOffset.x(),
                                  goalY + rotatedOffset.y(),
                                  m_GoalPosition.z() + rotatedOffset.z());
                goalWheels[i]->translate_to(wheelPos);
                goalWheels[i]->rotate_to(m_GoalRotation);
                goalWheels[i]->draw(&m_Program, context()->functions());
            }
        }

        m_cube->draw(&m_Program, context()->functions());


        if (mppi_state_lattice_flag && m_mppiInitialized) {

            int timesteps = m_mppiController->getNumTimesteps();
            float dt = m_mppiController->getDt();

            if (timesteps > 0 && dt > 0) {

                // Коэффициенты для преобразования costmap → мировые
                float cellSize = m_terrain->getWidth() / m_terrain->getCostmapWidth();
                float halfW = m_terrain->getWidth() / 2.0f;
                float halfH = m_terrain->getHeight() / 2.0f;

                int centerX = static_cast<int>((data.pos.x() + halfW) / cellSize);
                int centerZ = static_cast<int>((data.pos.z() + halfH) / cellSize);

                // Смещение локального окна относительно мировых координат
                float localOriginX = (centerX - 400) * cellSize - halfW;
                float localOriginZ = (centerZ - 400) * cellSize - halfH;

                std::vector<float> samples = m_mppiController->getSampledTrajectories();
                if (!samples.empty()) {
                    int valid_timesteps = timesteps - 1;  // последняя точка не имеет соединения
                    int num_traj = samples.size() / (valid_timesteps * 3);

                    // ========== 1. ОПТИМАЛЬНАЯ ТРАЕКТОРИЯ (синяя) - первая траектория (индекс 0) ==========
                    if (num_traj > 0) {
                        int opt_vertices_count = (valid_timesteps - 1) * 2;
                        GLfloat* opt_vertices = new GLfloat[opt_vertices_count * 3];
                        int vi = 0;

                        for (int t = 0; t < valid_timesteps - 1; t++) {
                            int idx0 = t * 3;      // первая траектория (индекс 0)
                            int idx1 = (t + 1) * 3;

                            // Локальные координаты траектории (из costmap) сразу преобразуем в мировые
                            float x0 = samples[idx0 + 0] * cellSize + localOriginX;
                            float y0 = samples[idx0 + 1];  // высота
                            float z0 = samples[idx0 + 2] * cellSize + localOriginZ;
                            float x1 = samples[idx1 + 0] * cellSize + localOriginX;
                            float y1 = samples[idx1 + 1];
                            float z1 = samples[idx1 + 2] * cellSize + localOriginZ;

                            opt_vertices[vi++] = x0;
                            opt_vertices[vi++] = y0;
                            opt_vertices[vi++] = z0;
                            opt_vertices[vi++] = x1;
                            opt_vertices[vi++] = y1;
                            opt_vertices[vi++] = z1;
                        }

                        m_mppiLines->Clear_Lines();
                        QVector3D darkBlue(0.0f, 0.2f, 0.8f);
                        m_mppiLines->Add_Lines(opt_vertices, darkBlue, vi / 3, 3);
                        delete[] opt_vertices;
                    }

                    // ========== 2. СЕМПЛЫ MPPI (красные) - остальные траектории ==========
                    if (num_traj > 1) {  // хотя бы 2 траектории (0-оптимальная, остальные - семплы)
                        // Пропускаем первую траекторию (индекс 0), берём с 1 по num_traj-1
                        int max_vertices = (num_traj - 1) * (valid_timesteps - 1) * 2 * 3;
                        GLfloat* samp_vertices = new GLfloat[max_vertices];
                        int vi = 0;

                        for (int i = 1; i < num_traj; i++) {  // начинаем с 1 (пропускаем оптимальную)
                            for (int t = 0; t < valid_timesteps - 1; t++) {
                                int idx0 = (i * valid_timesteps + t) * 3;
                                int idx1 = (i * valid_timesteps + t + 1) * 3;

                                // Преобразуем из локальных координат costmap в мировые
                                float x0 = samples[idx0 + 0] * cellSize + localOriginX;
                                float y0 = samples[idx0 + 1];
                                float z0 = samples[idx0 + 2] * cellSize + localOriginZ;
                                float x1 = samples[idx1 + 0] * cellSize + localOriginX;
                                float y1 = samples[idx1 + 1];
                                float z1 = samples[idx1 + 2] * cellSize + localOriginZ;

                                samp_vertices[vi++] = x0;
                                samp_vertices[vi++] = y0;
                                samp_vertices[vi++] = z0;
                                samp_vertices[vi++] = x1;
                                samp_vertices[vi++] = y1;
                                samp_vertices[vi++] = z1;
                            }
                        }

                        if (vi > 0) {
                            m_MultiLineRender->Clear_Lines();
                            QVector3D red(1.0f, 0.0f, 0.0f);
                            m_MultiLineRender->Add_Lines(samp_vertices, red, vi / 3, 4);
                        }
                        delete[] samp_vertices;
                    }

                    // Рисуем линии
                    m_mppiLines->draw_lines(&m_Program, context()->functions());
                    m_MultiLineRender->draw_lines(&m_Program, context()->functions());
                }
            }

        } else {
            m_pathLines->draw_lines(&m_Program, context()->functions());
            m_expLines->draw_lines(&m_Program, context()->functions());
        }


        for(auto o: m_Objects) o->draw(&m_Program, context()->functions());
        for(auto o: m_OBJ_Objects) o->draw(&m_Program, context()->functions());

        drawLocalWindowPerimeter(interpPos.x(), interpPos.z());
        m_perimeterLines->draw_lines(&m_Program, context()->functions());

        m_camera->draw(&m_Program);
        m_Program.release();
    }

    // ========== ОБЫЧНЫЙ РЕЖИМ ==========
    else {
        m_ProgramSkyBox.bind();
        m_ProgramSkyBox.setUniformValue("u_projectionMatrix", m_ProjectionMatrix);
        m_SkyBox->draw(&m_ProgramSkyBox, context()->functions());
        m_camera->draw(&m_ProgramSkyBox);
        m_ProgramSkyBox.release();

        m_Program.bind();
        m_Program.setUniformValue("u_projectionMatrix", m_ProjectionMatrix);
        m_Program.setUniformValue("p_id", 0);
        m_Program.setUniformValue("r_id", 0);
        m_Program.setUniformValue("u_map_offset", QVector3D(0, 0, 0));

        m_cube->draw(&m_Program, context()->functions());

        QQuaternion q_rot(interpOri.w(), interpOri.x(), interpOri.y(), interpOri.z());
        Truck->rotate_to(q_rot);
        Truck->translate_to(QVector3D(interpPos.x(), interpPos.y(), interpPos.z()));
        Truck->draw(&m_Program, context()->functions());

        for (int i = 0; i < 4; ++i) {
            if (!wheels[i]) continue;

            Eigen::Vector3f offset = Sim.cfg.wheelOffsets[i];
            offset.y() -= (Sim.cfg.wheel_radius - interpWheelPos[i]);

            Eigen::Vector3f rotatedOffset = interpOri * offset;

            QVector3D worldWheelPos(
                interpPos.x() + rotatedOffset.x(),
                interpPos.y() + rotatedOffset.y(),
                interpPos.z() + rotatedOffset.z()
            );
            wheels[i]->translate_to(worldWheelPos);

            QQuaternion wheelRot = q_rot;
            if (i < 2) {
                wheelRot = wheelRot * QQuaternion::fromAxisAndAngle(0, 1, 0, interpSteerAngle * RADTODEG);
            }
            wheelRot = wheelRot * QQuaternion::fromAxisAndAngle(1, 0, 0, interpWheelRotation[i] * RADTODEG);
            wheels[i]->rotate_to(wheelRot);

            wheels[i]->draw(&m_Program, context()->functions());
        }

        Terrain->draw(&m_Program, context()->functions());

        for(auto o: m_Objects) o->draw(&m_Program, context()->functions());
        for(auto o: m_OBJ_Objects) o->draw(&m_Program, context()->functions());

        m_camera->draw(&m_Program);
        m_Program.release();
    }

    // ========== ImGui (всегда) ==========
    if (m_showTuningWindow) {
        float k_min = 1000.0f, k_max = 500000.0f;
        float d_min = 1000.0f, d_max = 100000.0f;

        QtImGui::newFrame();
        ImGui::Begin("Suspension Tuning");

        // В блоке ImGui после "Suspension Tuning"
        const char* mode_str = "Manual";
        if (m_controllerMode == ControllerMode_MPPI) mode_str = "MPPI";
        else if (m_controllerMode == ControllerMode_RPP) mode_str = "RPP";
        else if (m_controllerMode == ControllerMode_VPC) mode_str = "VPC";
        ImGui::Text("Controller: %s", mode_str);

        ImGui::Text("Vehicle Mass: %.1f kg", Sim.cfg.mass);
        if (ImGui::Button("Auto-calculate damping")) Sim.updateDampingSpring();

        float avg_compression = (data.wheelPos[0] + data.wheelPos[1] + data.wheelPos[2] + data.wheelPos[3]) / 4.0f;
        float target_compression = Sim.cfg.susp_length / 2.0f;

        ImGui::Text("Avg compression: %.3f m / %.3f m (target)", avg_compression, target_compression);
        ImGui::ProgressBar(avg_compression / target_compression, ImVec2(-1, 20));

        ImGui::Text("Speed: %.1f m/s (set: %.1f)", m_noisy_local_vel.z(), m_v_setpoint);
        ImGui::ProgressBar(std::fabs(m_v_setpoint) / 20.0f, ImVec2(-1, 20));

        ImGui::Text("Yaw rate: %+.3f deg/s set: %+.3f deg/s",
                    m_noisy_yaw_rate * RADTODEG, m_w_setpoint * RADTODEG);

        float w_range = 1.0f;
        float w_norm = std::clamp(std::fabs(m_w_setpoint) / w_range, 0.0f, 1.0f);

        float bar_height = 20.0f;
        float half_width = ImGui::GetContentRegionAvail().x * 0.5f;
        ImVec2 bar_size = ImVec2(half_width, bar_height);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        ImU32 bg_color = ImGui::GetColorU32(ImGuiCol_FrameBg);
        ImU32 fill_color = ImGui::GetColorU32(ImGuiCol_PlotHistogram);

        ImVec2 left_pos = ImGui::GetCursorScreenPos();
        draw_list->AddRectFilled(left_pos,
                                 ImVec2(left_pos.x + half_width, left_pos.y + bar_height),
                                 bg_color);

        if (m_w_setpoint > 0.0f) {
            float fill_width = half_width * w_norm;
            draw_list->AddRectFilled(
                ImVec2(left_pos.x + half_width - fill_width, left_pos.y),
                ImVec2(left_pos.x + half_width, left_pos.y + bar_height),
                fill_color);
        }

        ImGui::Dummy(bar_size);
        ImGui::SameLine(0.0f, 0.0f);

        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.8f, 0.6f, 0.2f, 1.0f));
        if (m_w_setpoint >= 0.0f)
            ImGui::ProgressBar(0.0f, bar_size, "");
        else
            ImGui::ProgressBar(w_norm, bar_size, "");
        ImGui::PopStyleColor();

        ImGui::Text("%-6s %6s", "+1.0", "-1.0");
        ImGui::Separator();

        ImGui::SliderFloat("Spring k", &Sim.cfg.k_spring, k_min, k_max);
        ImGui::SliderFloat("Damping Comp", &Sim.cfg.damping_compression, d_min, d_max);
        ImGui::SliderFloat("Damping Rebound", &Sim.cfg.damping_rebound, d_min, d_max);

        ImGui::Separator();

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
        ImGui::Render();
        QtImGui::render();
    }


}

void OGLWidget::onDrawLines(GLfloat* vertices_buffer, QVector3D color_buffer,
                             unsigned long long counter, uint16_t index)
{
    // Важен порядок: сначала путь (index=1), потом экспансии (index=2)
    if (index == 1) {
        m_pathLines->Add_Lines(vertices_buffer, color_buffer, counter, 1);
    } else if (index == 2) {
        m_expLines->Add_Lines(vertices_buffer, color_buffer, counter, 2);
    }
}

void OGLWidget::onPlanningFinished(bool success, const Path& path,
                                    const std::vector<std::tuple<float, float, float>>& expansions,
                                    double planning_time_ms)
{
    static int plan_log_counter = 0;
       if (plan_log_counter++ % 10 == 0) {  // логируем каждый 10-й результат
           if (success) {
               qDebug() << "[OGLWidget] Path found:" << path.poses.size() << "poses in" << planning_time_ms << "ms";
           } else {
               qDebug() << "[OGLWidget] Planning failed after" << planning_time_ms << "ms";
           }
       }

       if (success) {
           Path world_path = path;
           float cellSize = m_terrain->getWidth() / m_terrain->getCostmapWidth();

           for (auto& pose : world_path.poses) {
               pose.position.x() = pose.position.x() * cellSize + m_stateLattice->localOffsetX;
               pose.position.y() = pose.position.y() * cellSize + m_stateLattice->localOffsetY;
           }

           if (m_rppController) {
               m_path_for_rpp = world_path;
               m_rppController->setPath(world_path);
           }
           if (m_vpcController) {
               m_vpcController->setPath(world_path);
           }
           m_pathNeedsReplan = false;
       }
}

void OGLWidget::mousePressEvent(QMouseEvent *event)
{
    // ЛЕВАЯ кнопка — установка цели в режиме CostMap
    if (event->button() == Qt::LeftButton && m_showCostMap) {
        m_GoalPosition = getPlaneCoordinatesFromMouse(QVector2D(event->pos()));

        // Берём текущий поворот машины и оставляем только yaw
        QQuaternion truckRot = Truck->getRotate();
        QVector3D euler = truckRot.toEulerAngles();
        m_GoalRotation = QQuaternion::fromEulerAngles(0, euler.y(), 0);

        m_GoalPlacing = true;
        qDebug() << "Goal placing started at:" << m_GoalPosition;

        event->accept();
        return;
    }

    // Средний клик — сброс цели
    if (event->button() == Qt::MidButton && m_showCostMap) {
        m_GoalPlacing = false;
        m_GoalPosition = QVector3D();
        m_MultiLineRender->Clear_Lines();
        m_pathLines->Clear_Lines();
        m_expLines->Clear_Lines();
        m_mppiLines->Clear_Lines();
        event->accept();
        return;
    }

    // Левая кнопка в обычном режиме — щуп
    if (event->buttons() == Qt::LeftButton && !m_showCostMap) {
        if (m_cameraMode == CameraMode_Free) {
            probeTerrain(event->pos().x(), event->pos().y());
        }
        event->accept();
        return;
    }

    // Правая кнопка — вращение камеры
    if (event->buttons() == Qt::RightButton) {
        lastX = event->localPos().x();
        lastY = event->localPos().y();
        camera = false;
    }

    event->accept();
}

std::vector<uint8_t> OGLWidget::extractLocalCostmap(int centerX, int centerZ, int windowSize) const {
    if (!m_terrain) return {};

    int fullW = m_terrain->getCostmapWidth();   // 4097
    int fullH = m_terrain->getCostmapHeight();  // 4097
    int halfWin = windowSize / 2;                // 400

    std::vector<uint8_t> localMap(windowSize * windowSize, 254);  // по умолчанию — стена

    const uint8_t* fullMap = m_terrain->getCostmapData();
    if (!fullMap) return localMap;

    for (int ly = 0; ly < windowSize; ++ly) {
        for (int lx = 0; lx < windowSize; ++lx) {
            int fx = centerX - halfWin + lx;
            int fy = centerZ - halfWin + ly;

            if (fx >= 0 && fx < fullW && fy >= 0 && fy < fullH) {
                localMap[ly * windowSize + lx] = fullMap[fy * fullW + fx];
            }
        }
    }

    return localMap;
}

void OGLWidget::drawLocalWindowPerimeter(float posX, float posZ) {

    float cellSize = m_terrain->getWidth() / m_terrain->getCostmapWidth();
    float halfW = m_terrain->getWidth() / 2.0f;
    float halfH = m_terrain->getHeight() / 2.0f;

    int centerX = static_cast<int>((posX + halfW) / cellSize);
    int centerZ = static_cast<int>((posZ + halfH) / cellSize);

    float left   = (centerX - 400) * cellSize - halfW;
    float right  = (centerX + 400) * cellSize - halfW;
    float top    = (centerZ - 400) * cellSize - halfH;
    float bottom = (centerZ + 400) * cellSize - halfH;

    GLfloat vertices[24] = {
        left,  0.51f, top,    right, 0.51f, top,
        right, 0.51f, top,    right, 0.51f, bottom,
        right, 0.51f, bottom, left,  0.51f, bottom,
        left,  0.51f, bottom, left,  0.51f, top
    };

    m_perimeterLines->Clear_Lines();
    QVector3D orange(1.0f, 0.5f, 0.0f);
    m_perimeterLines->Add_Lines(vertices, orange, 8, 1);
}

void OGLWidget::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton && m_GoalPlacing) {
        m_GoalPlacing = false;

        // Конвертируем мировые координаты в индексы costmap
        float cellSize = m_terrain->getWidth() / m_terrain->getCostmapWidth();
        float halfW = m_terrain->getWidth() / 2.0f;
        float halfH = m_terrain->getHeight() / 2.0f;

        int goalX = static_cast<int>((m_GoalPosition.x() + halfW) / cellSize);
        int goalZ = static_cast<int>((m_GoalPosition.z() + halfH) / cellSize);
        goalX = std::clamp(goalX, 0, m_terrain->getCostmapWidth() - 1);
        goalZ = std::clamp(goalZ, 0, m_terrain->getCostmapHeight() - 1);

        int startX = static_cast<int>((data.pos.x() + halfW) / cellSize);
        int startZ = static_cast<int>((data.pos.z() + halfH) / cellSize);
        startX = std::clamp(startX, 0, m_terrain->getCostmapWidth() - 1);
        startZ = std::clamp(startZ, 0, m_terrain->getCostmapHeight() - 1);

        // Вырезаем локальное окно 800×800 вокруг робота
        int centerX = startX;
        int centerZ = startZ;
        std::vector<uint8_t> localMap = extractLocalCostmap(centerX, centerZ, 800);

        // Координаты в локальном окне
        int localStartX = 400;  // центр окна
        int localStartZ = 400;
        int localGoalX = 400 + (goalX - startX);
        int localGoalZ = 400 + (goalZ - startZ);

        // Проверяем, что цель в пределах окна
        if (localGoalX < 0 || localGoalX >= 800 || localGoalZ < 0 || localGoalZ >= 800) {
            qWarning() << "Цель за пределами локального окна";
            return;
        }

        m_stateLattice->localOffsetX = (centerX - 400) * cellSize - halfW;
        m_stateLattice->localOffsetY = (centerZ - 400) * cellSize - halfH;

        // Обновляем планировщик локальной картой
        m_stateLattice->updateCostmapFromData(localMap.data());

        // В mouseReleaseEvent, замени:
        Eigen::Vector3f forward = data.ori * Eigen::Vector3f::UnitZ();
        float startYaw = std::atan2(forward.x(), forward.z());

        // goalYaw — угол из маркера (уже правильный):
        float goalYaw = qDegreesToRadians(m_GoalRotation.toEulerAngles().y());

        Pose start, goal;
        start.position = Eigen::Vector2f(localStartX, localStartZ);
        start.orientation = -startYaw + M_PI/2;
        goal.position = Eigen::Vector2f(localGoalX, localGoalZ);
        goal.orientation = -goalYaw + M_PI/2;

        m_stateLattice->asyncGetPathToTarget(start, goal);

        m_lastPlannedGoalPosition = m_GoalPosition;
        m_pathNeedsReplan = true;

        emit goalPositionSet(m_GoalPosition, m_GoalRotation);

        qDebug() << "Local window:" << localStartX << "," << localStartZ
                 << "->" << localGoalX << "," << localGoalZ
                 << "| yaw:" << (startYaw * RADTODEG) << "° ->" << (goalYaw * RADTODEG) << "°";
    }
}

void OGLWidget::mouseMoveEvent(QMouseEvent *event)
{

    // Вращение цели при зажатой левой кнопке в режиме CostMap
    if (m_GoalPlacing && (event->buttons() & Qt::LeftButton) && m_showCostMap) {
        QVector3D currentPlanePos = getPlaneCoordinatesFromMouse(QVector2D(event->pos()));
        QVector3D direction = currentPlanePos - m_GoalPosition;
        direction.setY(0);

        if (direction.lengthSquared() > 0.001f) {
            direction.normalize();
            m_GoalRotation = calculateLookAtRotation(
                        m_GoalPosition, m_GoalPosition + direction, m_GoalRotation);
        }
        event->accept();
        return;
    }

    if(event->buttons() == Qt::RightButton)
    {
        if (camera) {
            lastX = event->localPos().x();
            lastY = event->localPos().y();
            camera = false;
        }

        double xoffset = event->localPos().x() - lastX;
        double yoffset = lastY - event->localPos().y();

        lastX = event->localPos().x();
        lastY = event->localPos().y();

        if (m_cameraMode == CameraMode_Free) {
            m_camera->rotate_camera(xoffset, yoffset);
        } else {
            m_cameraOrbitYaw += static_cast<float>(xoffset) * 0.3f;
            m_cameraOrbitPitch += static_cast<float>(yoffset) * 0.3f;
            m_cameraOrbitPitch = std::clamp(m_cameraOrbitPitch, -10.0f, 60.0f);
        }
    }
    event->accept();
}

void OGLWidget::wheelEvent(QWheelEvent *event)
{
    if (m_cameraMode == CameraMode_Free) {
        if (event->angleDelta().y() > 0){
            m_camera->camera_zoom(true);
        }
        else{
            m_camera->camera_zoom(false);
        }
    } else {
        if (event->angleDelta().y() > 0)
            m_cameraDistance -= 1.0f;
        else
            m_cameraDistance += 1.0f;

        m_cameraDistance = std::clamp(m_cameraDistance, 5.0f, 50.0f);
    }
    event->accept();
}

void OGLWidget::probeTerrain(int screenX, int screenY) {
    if (!m_terrain) return;

    // Получаем позицию камеры и её направление
    QVector3D camPos = m_camera->getPosition();
    QVector3D camFront = m_camera->getFront();
    QVector3D camUp(0.0f, 1.0f, 0.0f);  // или m_camera->m_camera_up, если публичный

    // Строим viewMatrix
    QMatrix4x4 viewMatrix;
    viewMatrix.lookAt(camPos, camPos + camFront, camUp);

    QMatrix4x4 projMatrix = m_ProjectionMatrix;

    // Нормализованные координаты устройства
    float ndcX = (2.0f * screenX) / width() - 1.0f;
    float ndcY = 1.0f - (2.0f * screenY) / height();

    QVector4D nearPoint(ndcX, ndcY, -1.0f, 1.0f);
    QVector4D farPoint(ndcX, ndcY, 1.0f, 1.0f);

    QMatrix4x4 invProj = projMatrix.inverted();
    QMatrix4x4 invView = viewMatrix.inverted();

    QVector4D worldNear4 = invView * invProj * nearPoint;
    QVector4D worldFar4  = invView * invProj * farPoint;

    worldNear4 /= worldNear4.w();
    worldFar4  /= worldFar4.w();

    QVector3D rayOrigin = worldNear4.toVector3D();
    QVector3D rayDirection = (worldFar4.toVector3D() - rayOrigin).normalized();

    // Пересекаем луч с террейном (пошагово, как в примере RayCastPosition)
    const float step = 0.5f;
    const int maxSteps = 200;
    QVector3D currentPos = rayOrigin;

    qDebug() << "\n========== TERRAIN PROBE ==========";
    qDebug() << "Camera pos:" << camPos;
    qDebug() << "Ray origin:" << rayOrigin;
    qDebug() << "Ray direction:" << rayDirection;

    for (int i = 0; i < maxSteps; ++i) {
        currentPos += rayDirection * step;
        float terrainH = m_terrain->getHeight(currentPos.x(), currentPos.z());

        if (currentPos.y() <= terrainH) {
            // Луч упёрся в землю
            qDebug() << "HIT at step" << i;
            qDebug() << "World pos:" << currentPos.x() << currentPos.y() << currentPos.z();
            qDebug() << "Terrain height:" << terrainH;

            // Выводим окрестность 3x3
            float cellSize = m_terrain->getWidth() / m_terrain->getCostmapWidth();
            int cx = (int)((currentPos.x() / m_terrain->getWidth() + 0.5f) * m_terrain->getCostmapWidth());
            int cz = (int)((currentPos.z() / m_terrain->getHeight() + 0.5f) * m_terrain->getCostmapHeight());

            qDebug() << "Grid cell:" << cx << "," << cz;
            qDebug() << "Neighborhood 3x3 heights (meters):";
            for (int dy = -1; dy <= 1; ++dy) {
                QString line;
                for (int dx = -1; dx <= 1; ++dx) {
                    int nx = cx + dx;
                    int nz = cz + dy;
                    float wx = (nx / (float)m_terrain->getCostmapWidth() - 0.5f) * m_terrain->getWidth();
                    float wz = (nz / (float)m_terrain->getCostmapHeight() - 0.5f) * m_terrain->getHeight();
                    float nh = m_terrain->getHeight(wx, wz);
                    line += QString::number(nh, 'f', 2).rightJustified(8);
                }
                qDebug() << "  " << line.toStdString().c_str();
            }
            qDebug() << "===================================\n";
            return;
        }
    }

    qDebug() << "No hit within" << maxSteps << "steps";
    qDebug() << "===================================\n";
}

QVector3D OGLWidget::getPlaneCoordinatesFromMouse(const QVector2D &mousePos) {
    // Нормализованные координаты устройства
    QVector4D mouse_norm(2.0f * mousePos.x() / width() - 1.0f,
                         -2.0f * mousePos.y() / height() + 1.0f,
                         -1.0f, 1.0f);

    // Обратная проекция
    QMatrix4x4 invProj = m_ProjectionMatrix.inverted();
    QVector4D mouseInView = invProj * mouse_norm;
    QVector3D rayDir(mouseInView.x(), mouseInView.y(), -1.0f);
    rayDir.normalize();

    // Обратная видовая матрица
    QMatrix4x4 viewMatrix;
    QVector3D camPos = m_camera->getPosition();
    QVector3D camFront = m_camera->getFront();
    QVector3D camUp(0.0f, 1.0f, 0.0f);
    viewMatrix.lookAt(camPos, camPos + camFront, camUp);
    QMatrix4x4 invView = viewMatrix.inverted();

    QVector3D rayWorldDir = (invView * QVector4D(rayDir, 0.0f)).toVector3D().normalized();
    QVector3D camWorldPos = (invView * QVector4D(0, 0, 0, 1)).toVector3D();

    // Пересечение с плоскостью Y = costmapCarY
    const float planeY = 0.5f;
    QVector3D planeNormal(0.0f, 1.0f, 0.0f);
    QVector3D planePoint(0.0f, planeY, 0.0f);

    float t = QVector3D::dotProduct(planePoint - camWorldPos, planeNormal) /
              QVector3D::dotProduct(rayWorldDir, planeNormal);

    return camWorldPos + rayWorldDir * t;
}

QQuaternion OGLWidget::calculateLookAtRotation(const QVector3D& from,
                                               const QVector3D& to,
                                               const QQuaternion& currentRotation) {
    QVector3D direction = to - from;

    if (direction.lengthSquared() < 0.001f) {
        return currentRotation;
    }

    direction.normalize();

    QVector3D currentForward = currentRotation * QVector3D(0.0f, 0.0f, -1.0f);
    currentForward.setY(0);
    if (currentForward.lengthSquared() < 0.001f) {
        currentForward = QVector3D(0.0f, 0.0f, -1.0f);
    }
    currentForward.normalize();

    QVector3D axis = QVector3D::crossProduct(currentForward, direction);

    if (axis.lengthSquared() < 0.001f) {
        float dot = QVector3D::dotProduct(currentForward, direction);
        if (dot < -0.999f) {
            QQuaternion flip = QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0), 180.0f);
            return flip * currentRotation;
        }
        return currentRotation;
    }

    axis.normalize();

    float dot = QVector3D::dotProduct(currentForward, direction);
    float angle = qRadiansToDegrees(qAcos(qBound(-1.0f, dot, 1.0f)));

    QQuaternion deltaRotation = QQuaternion::fromAxisAndAngle(axis, angle);

    return deltaRotation * currentRotation;
}

void OGLWidget::keyPressEvent(QKeyEvent *event)
{
    // Tab — переключение costmap
    if (event->key() == Qt::Key_Tab) {
        m_showCostMap = !m_showCostMap;
        qDebug() << "CostMap mode:" << (m_showCostMap ? "ON" : "OFF");
        event->accept();
        return;
    }

    // F1 — переключение режима камеры
    if (event->key() == Qt::Key_F1) {
        m_cameraMode = (m_cameraMode == CameraMode_Free) ? CameraMode_Follow : CameraMode_Free;
        m_camForward = m_camBackward = m_camLeft = m_camRight = m_camUp = m_camDown = false;
        qDebug() << "Camera mode switched to:" << (m_cameraMode == CameraMode_Free ? "Free" : "Follow");
        event->accept();
        return;
    }

    // Управление машиной
    switch(event->key()) {
        case Qt::Key_W:
            keyW = true;
            break;
        case Qt::Key_S:
            keyS = true;
            break;
        case Qt::Key_A:
            keyA = true;
            break;
        case Qt::Key_D:
            keyD = true;
            break;
        case Qt::Key_Space:
            keySpace = true;
            break;
        case Qt::Key_Shift:
            keyShift = true;
            break;
        case Qt::Key_R: {
            Sim.reset();
            m_firstFrame = true;
            qDebug() << "Car respawned!";
            event->accept();
            break;
            }
    case Qt::Key_L: {
            mppi_state_lattice_flag = !mppi_state_lattice_flag;
            qDebug() << "Visualization mode:" << (mppi_state_lattice_flag ? "MPPI" : "StateLattice");
            event->accept();
            break;
            }
        case Qt::Key_F2: {
            m_showTuningWindow = !m_showTuningWindow;
            qDebug() << "Tuning window visible:" << m_showTuningWindow;
            event->accept();
            break;
            }
    case Qt::Key_M: {
        switch (m_controllerMode){
            case ControllerMode_Manual:
                m_controllerMode = ControllerMode_MPPI;
                qDebug() << "Mode: MPPI";
                break;
            case ControllerMode_MPPI:
                m_controllerMode = ControllerMode_RPP;
                qDebug() << "Mode: RPP";
                break;
            case ControllerMode_RPP:
                m_controllerMode = ControllerMode_VPC;
                qDebug() << "Mode: VPC";
                break;
            case ControllerMode_VPC:
                m_controllerMode = ControllerMode_Manual;
                m_v_setpoint = 0.0f;
                m_w_setpoint = 0.0f;
                qDebug() << "Mode: MANUAL";
                break;
        }
        event->accept();
        break;
    }
        default:
            QOpenGLWidget::keyPressEvent(event);
            break;
    }

    // Управление камерой в свободном режиме
    if (m_cameraMode == CameraMode_Free) {
        switch(event->key()) {
            case Qt::Key_W:
                m_camForward = true;
                break;
            case Qt::Key_S:
                m_camBackward = true;
                break;
            case Qt::Key_A:
                m_camLeft = true;
                break;
            case Qt::Key_D:
                m_camRight = true;
                break;
            case Qt::Key_Q:
                m_camUp = true;
                break;
            case Qt::Key_E:
                m_camDown = true;
                break;
        }
    }

    event->accept();
}

void OGLWidget::keyReleaseEvent(QKeyEvent *event)
{
    switch(event->key()) {
        case Qt::Key_W:
            keyW = false;
            break;
        case Qt::Key_S:
            keyS = false;
            break;
        case Qt::Key_A:
            keyA = false;
            break;
        case Qt::Key_D:
            keyD = false;
            break;
        case Qt::Key_Space:
            keySpace = false;
            break;
        case Qt::Key_Shift:
            keyShift = false;
            break;
        default:
            QOpenGLWidget::keyReleaseEvent(event);
            break;
    }

    if (m_cameraMode == CameraMode_Free) {
        switch(event->key()) {
            case Qt::Key_W:
                m_camForward = false;
                break;
            case Qt::Key_S:
                m_camBackward = false;
                break;
            case Qt::Key_A:
                m_camLeft = false;
                break;
            case Qt::Key_D:
                m_camRight = false;
                break;
            case Qt::Key_Q:
                m_camUp = false;
                break;
            case Qt::Key_E:
                m_camDown = false;
                break;
        }
    }

    event->accept();
}

void OGLWidget::initShaders()
{
    if(!m_Program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vertshader.vsh"))
        close();
    if(!m_Program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fragshader.fsh"))
        close();
    if(!m_Program.link())
        close();

    if(!m_ProgramSkyBox.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/skybox.vsh"))
        close();
    if(!m_ProgramSkyBox.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/skybox.fsh"))
        close();
    if(!m_ProgramSkyBox.link()) close();
}
