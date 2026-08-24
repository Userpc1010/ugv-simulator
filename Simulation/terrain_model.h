#ifndef TERRAIN_MODEL_H
#define TERRAIN_MODEL_H

#include <QString>
#include <QFile>
#include <QDebug>
#include <QImage>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include "simpleobject3d.h"
#include "costmapcolorizer.h"

struct PacejkaParams {
    float B = 5.0f;
    float C = 2.0f;
    float D = 1.0f;
    float E = 1.0f;
};

class TerrainModel {
public:
    TerrainModel(const QString &filename, int width, int height, float worldWidth, float worldHeight, float AltScale);
    ~TerrainModel();

    inline float getHeight(float x, float z) const {
        if (std::isnan(x) || std::isinf(x) || std::isnan(z) || std::isinf(z)) return 0.0f;

        float nx = (x / m_width) + 0.5f;
        float nz = (z / m_height) + 0.5f;

        if (nx < 0.0f || nx >= 1.0f || nz < 0.0f || nz >= 1.0f) return 0.0f;

        float fx = nx * float(m_cols_1);
        float fz = nz * float(m_rows_1);

        int x0 = int(fx);
        int z0 = int(fz);

        int x1 = std::min(x0 + 1, m_cols_1);
        int z1 = std::min(z0 + 1, m_rows_1);

        float tx = fx - x0;
        float tz = fz - z0;

        float h00 = m_heightValues[z0 * m_cols + x0];
        float h10 = m_heightValues[z0 * m_cols + x1];
        float h01 = m_heightValues[z1 * m_cols + x0];
        float h11 = m_heightValues[z1 * m_cols + x1];

        float interHeight = (h00 * (1.0f - tx) + h10 * tx) * (1.0f - tz) +
                             (h01 * (1.0f - tx) + h11 * tx) * tz;

        return interHeight * float(m_AltScale);
    }

    inline Eigen::Vector3f getNormal(float x, float z) const {
        if (std::isnan(x) || std::isinf(x) || std::isnan(z) || std::isinf(z))
            return Eigen::Vector3f::UnitY();

        float nx = (x / m_width) + 0.5f;
        float nz = (z / m_height) + 0.5f;

        if (nx < 0.0f || nx >= 1.0f || nz < 0.0f || nz >= 1.0f)
            return Eigen::Vector3f::UnitY();

        float fx = nx * float(m_cols_1);
        float fz = nz * float(m_rows_1);

        int x0 = int(fx);
        int z0 = int(fz);
        int x1 = std::min(x0 + 1, m_cols_1);
        int z1 = std::min(z0 + 1, m_rows_1);

        float tx = fx - x0;
        float tz = fz - z0;

        Eigen::Vector3f n00 = m_normals[z0 * m_cols + x0];
        Eigen::Vector3f n10 = m_normals[z0 * m_cols + x1];
        Eigen::Vector3f n01 = m_normals[z1 * m_cols + x0];
        Eigen::Vector3f n11 = m_normals[z1 * m_cols + x1];

        Eigen::Vector3f n0 = n00 * (1.0f - tx) + n10 * tx;
        Eigen::Vector3f n1 = n01 * (1.0f - tx) + n11 * tx;

        Eigen::Vector3f n = n0 * (1.0f - tz) + n1 * tz;
        n.normalize();

        return n;
    }

    inline uint8_t getCost(float x, float z) const {
        int c = (int)((x * m_invWidth + 0.5f) * m_cols_1);
        int r = (int)((z * m_invDepth + 0.5f) * m_rows_1);
        if (c < 0 || c >= m_cols || r < 0 || r >= m_rows) return 255;
        return m_costmap[r * m_cols + c];
    }

    PacejkaParams getZonePacejka(float x, float z) const;

    enum SurfaceZone {
            ZONE_IMPASSABLE = 0,   // горы, непроходимо
            ZONE_EARTH      = 1,   // базовая земля
            ZONE_FOREST     = 2,   // лес
            ZONE_MOUNTAIN   = 3,   // горная дорога
            ZONE_PASSAGES   = 4,   // подъёмы
            ZONE_SWAMP      = 5    // болото
        };

    bool loadSurfaceZones(const QString& earthPath,
                              const QString& forestPath,
                              const QString& mountainRoadPath,
                              const QString& passagesPath,
                              const QString& swampPath);

    SurfaceZone getSurfaceZone(float x, float z) const;
    float getZoneMu(float x, float z, const Eigen::Quaternionf& ori) const;
    float getZoneRollingResistance(float x, float z, const Eigen::Quaternionf& ori) const;
    float getZoneDrag(float x, float z) const;
    float getZoneYawDrag(float x, float z) const;

    // Новый метод: вычисление карты уклонов по методу Least Squares из статьи
    void computeSlopeMap(int estimationSize);

    // Новый метод: получение раскрашенной costmap на основе уклонов
    QImage getCostmapImage() const;

    // В public секции:
    void dumpHeightSample() const;

    // Геттеры для costmap (используются CostmapColorizer)
    const uint8_t* getCostmapData() const { return m_costmap; }
    int getCostmapWidth() const { return m_cols; }
    int getCostmapHeight() const { return m_rows; }

    // Геттеры физических размеров
    float getWidth() const { return m_width; }
    float getHeight() const { return m_height; }

private:
    int m_cols, m_rows, m_cols_1, m_rows_1;
    float m_width, m_height, m_invWidth, m_invDepth, m_AltScale;

    uint16_t* m_rawData = nullptr;
    float* m_heightValues = nullptr;    // нормализованные высоты (0..1)
    float* m_slopeValues = nullptr;     // уклоны (тангенс угла)
    uint8_t* m_costmap = nullptr;       // итоговая costmap (0 или 254)
    int m_slopeEstimationSize = 1;

    //Угол	Тангенс	Уклон %
    //20°	0.36	36%
    //25°	0.47	47%
    //30°	0.58	58%
    //35°	0.70	70%
    //45°	1.0	    100%

    // Параметры как в ROS Navigation
    const float maxSlope = 2.5f;             // тангенс ~45° — макс. уклон реалистично для внедорожника
    const float inscribed_radius = 4.2f;   // половина диагонали + padding
    const float inflation_radius = 8.4f;  // общий радиус как в state lattice
    const float cost_scaling_factor = 15.0f;

    Eigen::Vector3f* m_normals = nullptr;
    CostmapColorizer m_colorizer;

    void precomputeNormals();
    bool loadRAW16(const QString &filename, int width, int height);

    float tileHash(int ix, int iz) const;

    float getTileNoiseAdaptive(float x, float z, float baseCellSize, float noiseAmp) const;

    uint8_t* m_surfaceZones = nullptr;  // 4097x4097, значения SurfaceZone

    // Вспомогательный метод для Least Squares
    void getSlopeOfPoints(const std::vector<float>& heights,
                          const std::vector<float>& xs,
                          const std::vector<float>& zs,
                          float& slopeX, float& slopeZ) const;
};

#endif // TERRAIN_MODEL_H
