#ifndef STATELATTICE_H
#define STATELATTICE_H

#include <QObject>
#include <chrono>
#include "types.hpp"
#include "datatypes_smac_planner.h"
#include "smac_planner_lattice.hpp"
#include "oglwidget.h"
#include <memory>
#include <future>

class Costmap2D_Master;

class StateLattice : public QObject
{
    Q_OBJECT
public:

    StateLattice(QObject* parent = nullptr);
    ~StateLattice();

    void updateCostmapFromData(const uint8_t * data);
    void GetPathToTarget(Pose & start_pose, Pose & goal_pose, const uint8_t * costmap2D);
    void asyncGetPathToTarget(Pose start_pose, Pose goal_pose);
    bool isPlanningInProgress() const { return is_planning_in_progress_; }
    void cancelPlanning();

    // Методы для визуализации в OpenGL
    void drawExpansionsAsLines(const std::vector<std::tuple<float, float, float>>& expansions);
    void drawPathAsLines(const Path& path);

    // Очистка памяти
    void cleanup();

    float localOffsetX = 0.0f;
    float localOffsetY = 0.0f;

private:
    // Основной рабочий метод планировщика
    void runPlanning(Pose start_pose, Pose goal_pose);

signals:

    void planningStarted();
       void planningFinished(bool success, const Path& path,
                             const std::vector<std::tuple<float, float, float>>& expansions,
                             double planning_time_ms);
       void planningFailed(const QString& error);
       void planningCancelled();

       // Сигнал для OpenGL визуализации
       void DrawLines(GLfloat* vertices_buffer, QVector3D color_buffer, unsigned long long counter, uint16_t index);

private:

    // Планировщик и карта
    std::shared_ptr<Costmap2D_Master> costmap_master_;
    std::unique_ptr<SmacPlannerLattice> planner_;

    // Параметры
    SmacPlannerLatticeDynamicParams Dyn_planner_params_;
    SmacPlannerLatticeParams planner_params_;
    SmootherParams smoother_params_;
    CostmapConfig costmap_config_;

    GLfloat *output_vertices_close_node;
    uint32_t output_vertices_close_node_size = 0;
    uint8_t ptr_guard_output_vertices_close_node = 0;

    GLfloat *output_vertices_path;
    uint32_t output_vertices_path_size = 0;
    uint8_t ptr_guard_output_vertices_path = 0;

    const QVector3D green = QVector3D(0.0f, 1.0f, 0.0f);
    const QVector3D blue = QVector3D(0.0f, 0.0f, 1.0f);

    std::vector<std::tuple<float, float, float>> expansions_;

    // Флаги и синхронизация
    std::atomic<bool> is_planning_in_progress_{false};
    std::atomic<bool> should_cancel_{false};

    std::mutex costmap_mutex_;

    // Поток для асинхронного планирования
    std::future<void> planning_future_;

};

#endif // STATELATTICE_H
