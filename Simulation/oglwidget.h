#ifndef OGLWIDGET_H
#define OGLWIDGET_H

#include <QOpenGLWidget>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QBasicTimer>
#include <QTimer>
#include <algorithm>
#include <chrono>
#include "car_controller.h"
#include "simulation.h"
#include "terrain_model.h"
#include "simpleobject3d.h"
#include "QtImGui.h"
#include "ImGuiRenderer.h"
#include "costmap2d.h"
#include "multidrawline.h"
#include "datatypes_smac_planner.h"

namespace rpp { class RPPController; }
namespace vpc { class VectorPursuitController; }

class MPPIController;
class StateLattice;
class SimpleObject3D;
class Camera_3D;
class Cube;
class SkyBox;

class OGLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    enum CameraMode {
        CameraMode_Free,
        CameraMode_Follow
    };

    enum ControllerMode {
        ControllerMode_Manual,
        ControllerMode_MPPI,
        ControllerMode_RPP,
        ControllerMode_VPC
    };

    ControllerMode m_controllerMode = ControllerMode_Manual;

    OGLWidget(QWidget *parent = nullptr);
    ~OGLWidget();

    void probeTerrain(int screenX, int screenY);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
    void timerEvent(QTimerEvent* event);

    void initShaders();
    void updateFollowCamera(const Eigen::Vector3f& carPos, const Eigen::Quaternionf& carOri);
    QVector3D getPlaneCoordinatesFromMouse(const QVector2D &mousePos);
    QQuaternion calculateLookAtRotation(const QVector3D& from, const QVector3D& to, const QQuaternion& currentRotation);
    std::vector<uint8_t> extractLocalCostmap(int centerX, int centerZ, int windowSize = 800) const;
    void drawLocalWindowPerimeter(float posX, float posZ);

    void closeEvent(QCloseEvent *event);

signals:
    void goalPositionSet(QVector3D position, QQuaternion rotation);

public slots:
    void onDrawLines(GLfloat* vertices_buffer, QVector3D color_buffer,
                     unsigned long long counter, uint16_t index);
    void onPlanningFinished(bool success, const Path& path,
                            const std::vector<std::tuple<float, float, float>>& expansions,
                            double planning_time_ms);

private:
    QMatrix4x4 m_ProjectionMatrix;
    QOpenGLShaderProgram m_Program;
    QOpenGLShaderProgram m_ProgramSkyBox;
    QVector2D m_MousePosition;
    QQuaternion m_Rotation;
    QBasicTimer sim_timer;

    Simulation_Car Sim;
    TerrainModel *m_terrain = nullptr;

    CarController *m_controller;

    QVector<SimpleObject3D*> m_Objects;
    QVector<SimpleObject3D*> m_OBJ_Objects;
    SimpleObject3D * Truck = nullptr;
    SimpleObject3D * Truck_wheel_front_right = nullptr;
    SimpleObject3D * Truck_wheel_front_left = nullptr;
    SimpleObject3D * Truck_wheel_back_right = nullptr;
    SimpleObject3D * Truck_wheel_back_left = nullptr;
    SimpleObject3D * Terrain = nullptr;
    Camera_3D *m_camera;
    Cube *m_cube;
    SkyBox *m_SkyBox;
    MaterialLibrary m_MatLibrary;


    bool m_GoalPlacing = false;
    QVector3D m_GoalPosition;
    QQuaternion m_GoalRotation;

    // Модель цели (полная копия пикапа)
    SimpleObject3D* m_GoalTruck = nullptr;
    SimpleObject3D* m_GoalWheelFR = nullptr;
    SimpleObject3D* m_GoalWheelFL = nullptr;
    SimpleObject3D* m_GoalWheelRR = nullptr;
    SimpleObject3D* m_GoalWheelRL = nullptr;

    // Costmap
    CostmapRander *m_costmapRander = nullptr;
    MultiDrawLine *m_MultiLineRender = nullptr;
    bool m_showCostMap = false;

    // Линии для визуализации
    MultiDrawLine *m_pathLines = nullptr;   // путь StateLattice
    MultiDrawLine *m_expLines = nullptr;    // экспансии StateLattice
    MultiDrawLine *m_mppiLines = nullptr;   // будущий MPPI

    MultiDrawLine *m_perimeterLines = nullptr;  // периметр локального окна

    // Планировщик
    StateLattice *m_stateLattice = nullptr;

    bool mppi_state_lattice_flag = false;

    //VCP контроллер
    vpc::VectorPursuitController* m_vpcController = nullptr;

    //RPP контроллер
    rpp::RPPController* m_rppController = nullptr;

    Path m_path_for_rpp;

    // MPPI контроллер
    MPPIController* m_mppiController = nullptr;

    bool m_mppiInitialized = false;
    uint16_t mppi_conunter = 0;

    QVector3D m_lastPlannedGoalPosition;
    bool m_pathNeedsReplan = true;

    // Буфер для costmap (чтобы не выделять каждый кадр)
    std::vector<uint8_t> m_mppiCostmapBuffer;

    std::chrono::steady_clock::time_point lastTime;
    bool initialized = false;

    QVector3D m_smoothedCamPos;

    bool m_firstFrame = true;

    bool m_showTuningWindow = false;

    bool camera;

    double lastX = 0.0, lastY = 0.0;

    const float FIXED_DT = 0.01f;

    float m_noisy_yaw_rate = 0.0f;
    Eigen::Vector3f m_noisy_local_vel = Eigen::Vector3f::Zero();

    state_vec data;

    bool keyW = false;
    bool keyS = false;
    bool keyA = false;
    bool keyD = false;
    bool keySpace = false;
    bool keyShift = false;

    const float v_rate = 3.0f;
    const float w_rate = 0.5f;
    const float v_max = 20.0f;
    const float v_min = -10.0f;
    const float w_max = 1.0f;

    CameraMode m_cameraMode;

    state_vec m_prevData;
    float m_interpFactor = 0.0f;

    float m_v_setpoint = 0.0f;
    float m_w_setpoint = 0.0f;

    float m_cameraOrbitYaw;
    float m_cameraOrbitPitch;
    float m_cameraDistance;

    bool m_camForward = false;
    bool m_camBackward = false;
    bool m_camLeft = false;
    bool m_camRight = false;
    bool m_camUp = false;
    bool m_camDown = false;
};

#endif // OGLWIDGET_H
