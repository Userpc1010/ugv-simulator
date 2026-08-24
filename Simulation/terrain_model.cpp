#include "terrain_model.h"
#include <cstring>
#include <omp.h>

TerrainModel::TerrainModel(const QString &filename, int width, int height, float worldWidth, float worldHeight, float AltScale)
    : m_cols(width), m_rows(height),
      m_width(worldWidth), m_height(worldHeight),
      m_AltScale(AltScale)
{
    m_cols_1 = m_cols - 1;
    m_rows_1 = m_rows - 1;
    m_invWidth = 1.0f / m_width;
    m_invDepth = 1.0f / m_height;

    // Выделяем память
    m_rawData = new uint16_t[m_cols * m_rows];
    m_heightValues = new float[m_cols * m_rows];
    m_slopeValues = new float[m_cols * m_rows];
    m_costmap = new uint8_t[m_cols * m_rows];
    std::memset(m_costmap, 0, m_cols * m_rows);
    std::memset(m_slopeValues, 0, m_cols * m_rows * sizeof(float));

    // Загружаем RAW16 файл
    if (!loadRAW16(filename, width, height)) {
        qCritical() << "Failed to load RAW16 file:" << filename;
        for (int i = 0; i < m_cols * m_rows; i++) {
            m_rawData[i] = i % 65535;
            m_heightValues[i] = m_rawData[i] / 65535.0f;
        }
    }

   m_surfaceZones = new uint8_t[m_cols * m_rows];
   std::memset(m_surfaceZones, ZONE_IMPASSABLE, m_cols * m_rows);
}

TerrainModel::~TerrainModel() {
    delete[] m_normals;
    delete[] m_rawData;
    delete[] m_heightValues;
    delete[] m_slopeValues;
    delete[] m_costmap;
    delete[] m_surfaceZones;
}

bool TerrainModel::loadRAW16(const QString &filename, int width, int height) {
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        qCritical() << "Cannot open file:" << filename;
        return false;
    }

    qint64 expectedSize = (qint64)width * height * sizeof(uint16_t);
    qint64 fileSize = file.size();

    if (fileSize != expectedSize) {
        qCritical() << "RAW16 file size mismatch! Expected:" << expectedSize
                   << "Actual:" << fileSize;
        return false;
    }

    qint64 bytesRead = file.read(reinterpret_cast<char*>(m_rawData), expectedSize);
    if (bytesRead != expectedSize) {
        qCritical() << "Failed to read entire file. Read:" << bytesRead;
        return false;
    }

    file.close();

    for (int i = 0; i < m_cols * m_rows; i++) {
        m_heightValues[i] = m_rawData[i] / 65535.0f;
    }

    precomputeNormals();

    uint16_t minVal = 65535, maxVal = 0;
    for (int i = 0; i < m_cols * m_rows; i++) {
        if (m_rawData[i] < minVal) minVal = m_rawData[i];
        if (m_rawData[i] > maxVal) maxVal = m_rawData[i];
    }
    qDebug() << "RAW16 loaded: size" << width << "x" << height;
    qDebug() << "Min raw:" << minVal << "Max raw:" << maxVal;
    qDebug() << "Min height:" << minVal/65535.0f * m_AltScale
             << "Max height:" << maxVal/65535.0f * m_AltScale;

    return true;
}

void TerrainModel::precomputeNormals() {
    if (m_normals) delete[] m_normals;
    m_normals = new Eigen::Vector3f[m_cols * m_rows];

    float stepX = m_width / m_cols_1;
    float stepZ = m_height / m_rows_1;

    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            int c_prev = std::max(0, c - 1);
            int c_next = std::min(m_cols_1, c + 1);
            int r_prev = std::max(0, r - 1);
            int r_next = std::min(m_rows_1, r + 1);

            float hL = m_heightValues[r * m_cols + c_prev] * m_AltScale;
            float hR = m_heightValues[r * m_cols + c_next] * m_AltScale;
            float hD = m_heightValues[r_prev * m_cols + c] * m_AltScale;
            float hU = m_heightValues[r_next * m_cols + c] * m_AltScale;

            float dx = (hR - hL) / (stepX * (c_next - c_prev));
            float dz = (hU - hD) / (stepZ * (r_next - r_prev));

            Eigen::Vector3f n(-dx, 1.0f, -dz);
            m_normals[r * m_cols + c] = n.normalized();
        }
    }
    qDebug() << "Normals precomputed for" << m_cols << "x" << m_rows << "grid";
}

void TerrainModel::getSlopeOfPoints(const std::vector<float>& heights,
                                     const std::vector<float>& xs,
                                     const std::vector<float>& zs,
                                     float& slopeX, float& slopeZ) const {
    int n = heights.size();
    if (n < 3) {
        slopeX = 0.0f;
        slopeZ = 0.0f;
        return;
    }

    // Решаем нормальное уравнение (A^T * A) * x = A^T * B
    // A = [x_i, z_i, 1], B = [h_i]
    // Нам нужны только a (slopeX) и b (slopeZ)

    float sumX = 0, sumZ = 0, sumH = 0;
    float sumX2 = 0, sumZ2 = 0, sumXZ = 0;
    float sumXH = 0, sumZH = 0;

    for (int i = 0; i < n; ++i) {
        float x = xs[i];
        float z = zs[i];
        float h = heights[i];

        sumX += x;
        sumZ += z;
        sumH += h;
        sumX2 += x * x;
        sumZ2 += z * z;
        sumXZ += x * z;
        sumXH += x * h;
        sumZH += z * h;
    }

    // Строим систему 3x3 для [a, b, c]
    // | sumX2   sumXZ   sumX | | a |   | sumXH |
    // | sumXZ   sumZ2   sumZ | | b | = | sumZH |
    // | sumX    sumZ     n   | | c |   | sumH  |

     // Решаем через Крамера (для 3x3)
    float det = sumX2 * (sumZ2 * n - sumZ * sumZ)
              - sumXZ * (sumXZ * n - sumZ * sumX)
              + sumX  * (sumXZ * sumZ - sumZ2 * sumX);

    if (std::abs(det) < 1e-10f) {
        slopeX = 0.0f;
        slopeZ = 0.0f;
        return;
    }

    float detA = sumXH * (sumZ2 * n - sumZ * sumZ)
               - sumXZ * (sumZH * n - sumZ * sumH)
               + sumX  * (sumZH * sumZ - sumZ2 * sumH);

    float detB = sumX2 * (sumZH * n - sumZ * sumH)
               - sumXH * (sumXZ * n - sumZ * sumX)
               + sumX  * (sumXZ * sumH - sumZH * sumX);

    slopeX = detA / det;
    slopeZ = detB / det;
}

void TerrainModel::computeSlopeMap(int estimationSize) {
    m_slopeEstimationSize = std::max(1, estimationSize);

    float cellSizeX = m_width / m_cols_1;
    float cellSizeZ = m_height / m_rows_1;
    float rawToMeters = m_AltScale / 65535.0f;  // конвертация raw → метры

    qDebug() << "Computing slope map from raw data with estimation size" << m_slopeEstimationSize
             << "for" << m_cols << "x" << m_rows << "grid...";

    float* localSlope = new float[m_cols * m_rows];

    #pragma omp parallel for collapse(2) if(m_cols * m_rows > 10000)
    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            // === Локальный уклон (SA=1, окно 3×3) из raw данных ===
            std::vector<float> hl, xl, zl;
            uint16_t raw_min = 65535, raw_max = 0;

            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    int nr = r + dr;
                    int nc = c + dc;
                    if (nr >= 0 && nr < m_rows && nc >= 0 && nc < m_cols) {
                        uint16_t raw = m_rawData[nr * m_cols + nc];
                        float h = raw * rawToMeters;
                        hl.push_back(h);
                        xl.push_back(dc * cellSizeX);
                        zl.push_back(dr * cellSizeZ);

                        if (raw < raw_min) raw_min = raw;
                        if (raw > raw_max) raw_max = raw;
                    }
                }
            }

            // Локальный уклон через Least Squares
            float sx, sz;
            getSlopeOfPoints(hl, xl, zl, sx, sz);
            localSlope[r * m_cols + c] = std::sqrt(sx * sx + sz * sz);

            // Размах высот в окне 3×3
            float heightRange = (raw_max - raw_min) * rawToMeters /
                                std::max(cellSizeX, cellSizeZ);

            // === Глобальный уклон (SA=m_slopeEstimationSize) ===
            if (m_slopeEstimationSize > 1) {
                std::vector<float> hg, xg, zg;
                for (int dr = -m_slopeEstimationSize; dr <= m_slopeEstimationSize; ++dr) {
                    for (int dc = -m_slopeEstimationSize; dc <= m_slopeEstimationSize; ++dc) {
                        int nr = r + dr;
                        int nc = c + dc;
                        if (nr >= 0 && nr < m_rows && nc >= 0 && nc < m_cols) {
                            hg.push_back(m_rawData[nr * m_cols + nc] * rawToMeters);
                            xg.push_back(dc * cellSizeX);
                            zg.push_back(dr * cellSizeZ);
                        }
                    }
                }
                float gx, gz;
                getSlopeOfPoints(hg, xg, zg, gx, gz);
                float globalSlope = std::sqrt(gx * gx + gz * gz);

                // Итоговый уклон = max(локальный, глобальный, размах высот)
                m_slopeValues[r * m_cols + c] = std::max({localSlope[r * m_cols + c],
                                                           globalSlope,
                                                           heightRange});
            } else {
                m_slopeValues[r * m_cols + c] = std::max(localSlope[r * m_cols + c], heightRange);
            }
        }
    }

    delete[] localSlope;

    qDebug() << "Slope map computed from raw data.";

    // Статистика
    // Статистика
    float minVal = m_slopeValues[0];
    float maxVal = m_slopeValues[0];
    float sumVal = 0.0;
    int lethalCount = 0;      // > maxSlope
    int highCostCount = 0;    // 0.5*maxSlope .. maxSlope
    int mediumCostCount = 0;  // 0.2*maxSlope .. 0.5*maxSlope
    int lowCostCount = 0;     // 0.1 .. 0.2*maxSlope
    int flatCount = 0;        // < 0.1

    for (int i = 0; i < m_cols * m_rows; ++i) {
        float s = m_slopeValues[i];
        if (s < minVal) minVal = s;
        if (s > maxVal) maxVal = s;
        sumVal += s;

        if (s > maxSlope) {
            lethalCount++;
        } else if (s > 0.5f * maxSlope) {
            highCostCount++;
        } else if (s > 0.2f * maxSlope) {
            mediumCostCount++;
        } else if (s > 0.1f) {
            lowCostCount++;
        } else {
            flatCount++;
        }
    }

    qDebug() << "Slope statistics - Min:" << minVal << "Max:" << maxVal
             << "Avg:" << (sumVal / (m_cols * m_rows));
    qDebug() << "Lethal (> " << maxSlope << " ≈" << (atan(maxSlope) * 180.0f / M_PI) << "°):"
             << lethalCount << "(" << (100.0f * lethalCount / (m_cols * m_rows)) << "%)";
    qDebug() << "High cost (" << (0.5f * maxSlope) << ".." << maxSlope << "):"
             << highCostCount << "(" << (100.0 * highCostCount / (m_cols * m_rows)) << "%)";
    qDebug() << "Medium cost (" << (0.2f * maxSlope) << ".." << (0.5f * maxSlope) << "):"
             << mediumCostCount << "(" << (100.0 * mediumCostCount / (m_cols * m_rows)) << "%)";
    qDebug() << "Low cost (0.1.." << (0.2f * maxSlope) << "):"
             << lowCostCount << "(" << (100.0f * lowCostCount / (m_cols * m_rows)) << "%)";
    qDebug() << "Flat (< 0.1 ≈5.7°):"
             << flatCount << "(" << (100.0f * flatCount / (m_cols * m_rows)) << "%)";
}

void TerrainModel::dumpHeightSample() const {
    qDebug() << "\n========== TERRAIN HEIGHT SAMPLE ==========";
    qDebug() << "Map size:" << m_cols << "x" << m_rows;
    qDebug() << "Cell size:" << m_width / m_cols_1 << "x" << m_height / m_rows_1 << "meters";
    qDebug() << "AltScale:" << m_AltScale;

    // Выбираем несколько характерных точек (центр, углы, середина сторон)
    struct SamplePoint {
        int x, y;
        const char* name;
    };

    SamplePoint samples[] = {
        {m_cols/2, m_rows/2, "Center"},
        {m_cols/4, m_rows/4, "Quarter"},
        {3*m_cols/4, 3*m_rows/4, "ThreeQuarter"},
        {m_cols/2, m_rows/4, "North"},
        {m_cols/4, m_rows/2, "West"},
        {3*m_cols/4, m_rows/2, "East"},
        {m_cols/2, 3*m_rows/4, "South"},
        {100, 100, "Point_100_100"},
        {500, 500, "Point_500_500"},
        {1000, 1000, "Point_1000_1000"},
        {2000, 2000, "Point_2000_2000"},
        {3000, 3000, "Point_3000_3000"}
    };

    for (const auto& sp : samples) {
        if (sp.x >= 0 && sp.x < m_cols && sp.y >= 0 && sp.y < m_rows) {
            int idx = sp.y * m_cols + sp.x;
            float h_norm = m_heightValues[idx];
            float h_real = h_norm * m_AltScale;
            float slope = m_slopeValues ? m_slopeValues[idx] : -1.0f;

            qDebug() << "\n---" << sp.name << "(x=" << sp.x << ", y=" << sp.y << ") ---";
            qDebug() << "  Height (normalized):" << h_norm;
            qDebug() << "  Height (meters):" << h_real;
            qDebug() << "  Slope:" << slope << "(tan =" << slope << ", angle =" << (atan(slope) * 180.0 / M_PI) << "°)";

            // Выводим окрестность 5x5
            qDebug() << "  Neighborhood 5x5 heights (meters):";
            QString line;
            for (int dy = -2; dy <= 2; ++dy) {
                line.clear();
                for (int dx = -2; dx <= 2; ++dx) {
                    int nx = sp.x + dx;
                    int ny = sp.y + dy;
                    if (nx >= 0 && nx < m_cols && ny >= 0 && ny < m_rows) {
                        float h = m_heightValues[ny * m_cols + nx] * m_AltScale;
                        line += QString::number(h, 'f', 1).rightJustified(7);
                    } else {
                        line += "   -   ";
                    }
                }
                qDebug() << "   " << line.toStdString().c_str();
            }
        }
    }

    qDebug() << "\n==========================================\n";
}

QImage TerrainModel::getCostmapImage() const {
    if (!m_slopeValues) {
        return QImage(m_cols, m_rows, QImage::Format_RGBA8888);
    }

    float cellSize = m_width / m_cols_1;
    float ins_cells = inscribed_radius / cellSize;
    float inf_cells = inflation_radius / cellSize;
    int radius = static_cast<int>(std::ceil(inf_cells));

    uint8_t* tempCostmap = new uint8_t[m_cols * m_rows];
    uint8_t* original = new uint8_t[m_cols * m_rows];

    // Шаг 1: Градиент стоимости вместо бинарной карты
    #pragma omp parallel for if(m_cols * m_rows > 10000)
    for (int i = 0; i < m_cols * m_rows; ++i) {
        float slope = m_slopeValues[i];

        if (slope > maxSlope) {
            tempCostmap[i] = 254;  // летальное препятствие (>30°)
        } else if (slope < 0.1f) {
            tempCostmap[i] = 0;    // практически ровно (<5.7°)
        } else {
            // Градиент: 1..252 пропорционально уклону от 0.1 до maxSlope
            float normalized = (slope - 0.1f) / (maxSlope - 0.1f);
            int cost = static_cast<int>(252.0f * normalized);
            tempCostmap[i] = static_cast<uint8_t>(std::clamp(cost, 1, 252));
        }
    }

    // Шаг 2: Летальная рамка в 1 пиксель по периметру карты
    #pragma omp parallel for if(m_cols * m_rows > 10000)
    for (int y = 0; y < m_rows; ++y) {
        for (int x = 0; x < m_cols; ++x) {
            if (x == 0 || x == m_cols - 1 || y == 0 || y == m_rows - 1) {
                tempCostmap[x + y * m_cols] = 254;
            }
        }
    }

    std::memcpy(original, tempCostmap, m_cols * m_rows);

    // Шаг 3: Инфляция только для летальных препятствий (254) и высокой стоимости (>200)
    #pragma omp parallel for schedule(dynamic) if(m_cols * m_rows > 10000)
    for (int y = 0; y < m_rows; ++y) {
        for (int x = 0; x < m_cols; ++x) {
            // Инфляцию делаем только от летальных и очень дорогих ячеек
            if (original[y * m_cols + x] >= 200) {
                for (int iy = -radius; iy <= radius; ++iy) {
                    for (int ix = -radius; ix <= radius; ++ix) {
                        int nx = x + ix;
                        int ny = y + iy;
                        if (nx < 0 || nx >= m_cols || ny < 0 || ny >= m_rows) continue;

                        float dist = std::sqrt(ix*ix + iy*iy);
                        if (dist > inf_cells) continue;

                        unsigned char cost;
                        if (dist <= ins_cells) {
                            cost = 253;  // зона гарантированного столкновения
                        } else {
                            float dist_rel = dist - ins_cells;
                            float max_dist = inf_cells - ins_cells;
                            if (max_dist < 1e-6f) max_dist = 1e-6f;
                            // Экспоненциальное затухание стоимости от 252 до 1
                            int val = static_cast<int>(252.0f * std::exp(-cost_scaling_factor * (dist_rel / max_dist)));
                            cost = static_cast<unsigned char>(std::clamp(val, 1, 252));
                        }

                        int idx = ny * m_cols + nx;
                        if (cost > tempCostmap[idx]) {
                            tempCostmap[idx] = cost;
                        }
                    }
                }
            }
        }
    }

    delete[] original;

    // Сохраняем в m_costmap
    std::memcpy(m_costmap, tempCostmap, m_cols * m_rows);

    QImage result = m_colorizer.colorizeCostmap(tempCostmap, m_cols, m_rows);
    delete[] tempCostmap;

    return result;
}

bool TerrainModel::loadSurfaceZones(const QString& earthPath,
                                     const QString& forestPath,
                                     const QString& mountainRoadPath,
                                     const QString& passagesPath,
                                     const QString& swampPath)
{
    // Порядок загрузки: Earth → Forest → Mountain road → Passages → Swamp
    struct ZoneFile {
        QString path;
        SurfaceZone zone;
    };

    ZoneFile zones[] = {
        { earthPath,        ZONE_EARTH },
        { forestPath,       ZONE_FOREST },
        { mountainRoadPath, ZONE_MOUNTAIN },
        { passagesPath,     ZONE_PASSAGES },
        { swampPath,        ZONE_SWAMP }
    };

    for (const auto& zf : zones) {
        QImage img(zf.path);
        if (img.isNull()) {
            qCritical() << "Cannot load surface zone:" << zf.path;
            return false;
        }

        img = img.convertToFormat(QImage::Format_RGB888);
        if (img.width() != m_cols || img.height() != m_rows) {
            qCritical() << "Zone size mismatch:" << zf.path
                        << "expected" << m_cols << "x" << m_rows
                        << "got" << img.width() << "x" << img.height();
            return false;
        }

        // Порог белого
        const int threshold = 128;

        for (int y = 0; y < m_rows; ++y) {
            const uchar* scanline = img.constScanLine(y);
            for (int x = 0; x < m_cols; ++x) {
                // 24-bit: берём красный канал (для ч/б все каналы одинаковы)
                uchar r = scanline[x * 3];
                if (r > threshold) {
                    m_surfaceZones[y * m_cols + x] = zf.zone;
                }
            }
        }

        qDebug() << "Loaded zone" << zf.path << "as ID" << (int)zf.zone;
    }

    qDebug() << "Surface zones loaded. Total zones:" << 5;
    return true;
}

TerrainModel::SurfaceZone TerrainModel::getSurfaceZone(float x, float z) const
{
    if (!m_surfaceZones) return ZONE_IMPASSABLE;

    if (std::isnan(x) || std::isinf(x) || std::isnan(z) || std::isinf(z))
        return ZONE_IMPASSABLE;

    int px = static_cast<int>((x / m_width + 0.5f) * m_cols);
    int py = static_cast<int>((z / m_height + 0.5f) * m_rows);

    px = std::clamp(px, 0, m_cols - 1);
    py = std::clamp(py, 0, m_rows - 1);

    return static_cast<SurfaceZone>(m_surfaceZones[py * m_cols + px]);
}

float TerrainModel::tileHash(int ix, int iz) const
{
    unsigned int h = ix * 374761393u + iz * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h = h ^ (h >> 16);
    return (h & 0xFFFF) / 65535.0f;
}

float TerrainModel::getTileNoiseAdaptive(float x, float z, float baseCellSize, float noiseAmp) const
{
    // Случайный размер ячейки: от 0.1м до 5.5м
    float cellSize = baseCellSize * (0.5f + tileHash((int)(x * 0.1f), (int)(z * 0.1f)));
    cellSize = std::clamp(cellSize, 0.1f, 5.5f);

    // Случайная резкость: 0 = резкий, 1 = плавный
    float sharpness = tileHash((int)(x * 0.05f), (int)(z * 0.05f));

    float fx = x / cellSize;
    float fz = z / cellSize;

    int x0 = (int)std::floor(fx);
    int z0 = (int)std::floor(fz);

    float tx = fx - x0;
    float tz = fz - z0;

    float h00 = tileHash(x0, z0);
    float h10 = tileHash(x0 + 1, z0);
    float h01 = tileHash(x0, z0 + 1);
    float h11 = tileHash(x0 + 1, z0 + 1);

    // Резкий переход
    if (sharpness < 0.5f) {
        if (tx < 0.5f && tz < 0.5f) return h00;
        if (tx >= 0.5f && tz < 0.5f) return h10;
        if (tx < 0.5f && tz >= 0.5f) return h01;
        return h11;
    }

    // Плавный переход (smoothstep)
    if (sharpness > 0.5f) {
        tx = tx * tx * (3.0f - 2.0f * tx);
        tz = tz * tz * (3.0f - 2.0f * tz);
    }

    float h0 = h00 * (1.0f - tx) + h10 * tx;
    float h1 = h01 * (1.0f - tx) + h11 * tx;

    return h0 * (1.0f - tz) + h1 * tz;
}

float TerrainModel::getZoneMu(float x, float z, const Eigen::Quaternionf& ori) const
{
    SurfaceZone zone = getSurfaceZone(x, z);

    float baseMu;
    float noiseAmp;

    switch (zone) {
        case ZONE_EARTH:    baseMu = 2.0f;  noiseAmp = 0.10f; break;
        case ZONE_FOREST:   baseMu = 1.8f;  noiseAmp = 0.20f; break;
        case ZONE_MOUNTAIN: baseMu = 1.55f; noiseAmp = 0.55f; break;
        case ZONE_PASSAGES: baseMu = 1.45f; noiseAmp = 0.65f; break;
        case ZONE_SWAMP:    baseMu = 1.1f;  noiseAmp = 1.6f;  break;
        default:            baseMu = 1.0f;  noiseAmp = 1.1f;  break;
    }

    float hash;

    if (zone == ZONE_SWAMP) {

        hash = getTileNoiseAdaptive(x, z, 1.0f, noiseAmp);

    } else if (zone == ZONE_PASSAGES) {
        Eigen::Vector3f n = getNormal(x, z);

        // Направление вверх по склону (горизонтальная часть нормали)
        Eigen::Vector3f upSlope(n.x(), 0.0f, n.z());
        upSlope.normalize();

        // Направление машины (куда смотрит капот)
        Eigen::Vector3f forward = ori * Eigen::Vector3f::UnitZ();
        forward.y() = 0;
        forward.normalize();

        float dot = forward.dot(upSlope);

        // Обычный синусный шум
        hash = std::fmod(std::sin(x * 39.731f + z * 17.291f) * 12543.1793f, 1.0f);
        if (hash < 0.0f) hash += 1.0f;

        // Плавная модификация
        baseMu *= (1.0f + dot * 0.1f);
        noiseAmp *= (1.0f - std::abs(dot) * 0.3f);
        hash = 0.5f + (hash - 0.5f) * (1.0f - std::abs(dot) * 0.3f);

    } else {
        hash = std::fmod(std::sin(x * 12.9898f + z * 78.233f) * 43758.5453f, 1.0f);
        if (hash < 0.0f) hash += 1.0f;
    }

    float noise = (hash - 0.5f) * 2.0f * noiseAmp;
    return std::max(0.0f, baseMu + noise);
}

float TerrainModel::getZoneRollingResistance(float x, float z, const Eigen::Quaternionf& ori) const
{
    SurfaceZone zone = getSurfaceZone(x, z);

    float baseRR;
    float noiseAmp;

    switch (zone) {
        case ZONE_EARTH:    baseRR = 10.0f;  noiseAmp = 2.0f;  break;
        case ZONE_FOREST:   baseRR = 25.0f;  noiseAmp = 15.0f;  break;
        case ZONE_MOUNTAIN: baseRR = 30.0f;  noiseAmp = 25.0f;  break;
        case ZONE_PASSAGES: baseRR = 35.0f;  noiseAmp = 45.0f; break;
        case ZONE_SWAMP:    baseRR = 70.0f;  noiseAmp = 65.0f; break;
        default:            baseRR = 100.0f; noiseAmp = 10.0f; break;
    }

    float hash;

    if (zone == ZONE_SWAMP) {

        float trapHash = getTileNoiseAdaptive(x, z, 2.5f, 1.0f);
        if (trapHash < 0.2f) {
            baseRR = 550.0f;  // трясина — огромное сопротивление
            noiseAmp = 120.0f;
        } else {
            baseRR = 70.0f;   // обычное болото
            noiseAmp = 35.0f;
        }
        hash = getTileNoiseAdaptive(x, z, 0.7f, noiseAmp);

    } else if (zone == ZONE_PASSAGES) {
        Eigen::Vector3f n = getNormal(x, z);

        Eigen::Vector3f upSlope(n.x(), 0.0f, n.z());
        upSlope.normalize();

        Eigen::Vector3f forward = ori * Eigen::Vector3f::UnitZ();
        forward.y() = 0;
        forward.normalize();

        float dot = forward.dot(upSlope);

        hash = std::fmod(std::sin(x * 39.731f + z * 17.291f) * 12543.1793f, 1.0f);
        if (hash < 0.0f) hash += 1.0f;

        baseRR *= (1.0f - dot * 0.4f);
        noiseAmp *= (1.0f - std::abs(dot) * 0.3f);
        hash = 0.5f + (hash - 0.5f) * (1.0f - std::abs(dot) * 0.2f);

    } else {
        hash = std::fmod(std::sin(x * 39.731f + z * 17.291f) * 12543.1793f, 1.0f);
        if (hash < 0.0f) hash += 1.0f;
    }

    float noise = (hash - 0.5f) * 2.0f * noiseAmp;
    return std::max(0.0f, baseRR + noise);
}

float TerrainModel::getZoneDrag(float x, float z) const
{
    SurfaceZone zone = getSurfaceZone(x, z);

    float baseDrag;
    float noiseAmp;

    switch (zone) {
        case ZONE_EARTH:    baseDrag = 0.1f;  noiseAmp = 0.05f;  break;
        case ZONE_FOREST:   baseDrag = 0.2f;  noiseAmp = 15.0f;  break;
        case ZONE_MOUNTAIN: baseDrag = 0.35f; noiseAmp = 40.0f;  break;
        case ZONE_PASSAGES: baseDrag = 0.45f; noiseAmp = 80.0f;  break;
        case ZONE_SWAMP:
            if (getTileNoiseAdaptive(x, z, 2.5f, 1.0f) < 0.2f) {
                baseDrag = 7500.0f;  // трясина
                noiseAmp = 1000.0f;
            } else {
                baseDrag = 550.0f;  // обычное болото
                noiseAmp = 40.0f;
            }
            break;
        default:  baseDrag = 1.0f; noiseAmp = 1.0f; break;
    }

    // Шум с уникальным seed
    float hash = std::fmod(std::sin(x * 53.311f + z * 91.177f) * 24657.9135f, 1.0f);
    if (hash < 0.0f) hash += 1.0f;

    float noise = (hash - 0.5f) * 2.0f * noiseAmp;
    return std::max(0.0f, baseDrag + noise);
}

float TerrainModel::getZoneYawDrag(float x, float z) const
{
    SurfaceZone zone = getSurfaceZone(x, z);

    float baseYawDrag;
    float noiseAmp;

    switch (zone) {
        case ZONE_EARTH:    baseYawDrag = 0.1f;  noiseAmp = 0.05f;  break;
        case ZONE_FOREST:   baseYawDrag = 0.2f;  noiseAmp = 5.0f;  break;
        case ZONE_MOUNTAIN: baseYawDrag = 0.35f;  noiseAmp = 10.0f; break;
        case ZONE_PASSAGES: baseYawDrag = 0.40f;  noiseAmp = 15.0f; break;
        case ZONE_SWAMP:
            if (getTileNoiseAdaptive(x, z, 2.5f, 1.0f) < 0.2f) {
                baseYawDrag = 5500.0f;  // трясина
                noiseAmp = 800.0f;
            } else {
                baseYawDrag = 300.0f;   // обычное болото
                noiseAmp = 80.0f;
            }
            break;
        default: baseYawDrag = 1.0f; noiseAmp = 1.0f; break;
    }

    // Шум с другим уникальным seed
    float hash = std::fmod(std::sin(x * 73.171f + z * 37.519f) * 15873.4271f, 1.0f);
    if (hash < 0.0f) hash += 1.0f;

    float noise = (hash - 0.5f) * 2.0f * noiseAmp;
    return std::max(0.0f, baseYawDrag + noise);
}

PacejkaParams TerrainModel::getZonePacejka(float x, float z) const
{
    PacejkaParams p;

    SurfaceZone zone = getSurfaceZone(x, z);

    float baseB, baseC, baseD, baseE;
    float ampB, ampC, ampD, ampE;

    switch (zone) {
        case ZONE_EARTH:
            baseB = 5.0f; baseC = 2.0f; baseD = 1.1f; baseE = 1.0f;
            ampB  = 0.3f; ampC  = 0.1f; ampD  = 0.05f; ampE = 0.1f;
            break;
        case ZONE_FOREST:
            baseB = 4.0f; baseC = 2.5f; baseD = 1.0f; baseE = 0.9f;
            ampB  = 0.5f; ampC  = 0.15f; ampD = 0.1f; ampE  = 0.2f;
            break;
        case ZONE_MOUNTAIN:
            baseB = 3.9f; baseC = 1.5f; baseD = 1.0f; baseE = 1.1f;
            ampB  = 0.7f; ampC  = 0.2f; ampD  = 0.15f; ampE = 0.3f;
            break;
        case ZONE_PASSAGES:
            baseB = 2.1f; baseC = 1.3f; baseD = 0.9f; baseE = 1.3f;
            ampB  = 0.8f; ampC  = 0.25f; ampD = 0.2f; ampE  = 0.35f;
            break;
        case ZONE_SWAMP:
            baseB = 9.0f; baseC = 1.2f; baseD = 0.5f; baseE = 0.7f;
            ampB  = 2.5f; ampC  = 0.5f; ampD  = 0.4f; ampE  = 0.6f;
            break;
        default:
            baseB = 10.0f; baseC = 1.0f; baseD = 1.0f; baseE = 1.0f;
            ampB  = 0.0f; ampC  = 0.0f; ampD  = 0.0f; ampE  = 0.0f;
            break;
    }

    float h1, h2, h3, h4;

    if (zone == ZONE_SWAMP) {
        // Адаптивный тайловый шум для болота
        h1 = getTileNoiseAdaptive(x, z, 0.6f, 1.0f);
        h2 = getTileNoiseAdaptive(x, z, 0.8f, 1.0f);
        h3 = getTileNoiseAdaptive(x, z, 0.5f, 1.0f);
        h4 = getTileNoiseAdaptive(x, z, 0.9f, 1.0f);
    } else {
        // Обычный синусный шум для остальных зон
        h1 = std::fmod(std::sin(x * 12.9898f + z * 78.233f) * 43758.5453f, 1.0f);
        h2 = std::fmod(std::sin(x * 39.731f + z * 17.291f) * 12543.1793f, 1.0f);
        h3 = std::fmod(std::sin(x * 67.123f + z * 45.891f) * 34821.7731f, 1.0f);
        h4 = std::fmod(std::sin(x * 91.571f + z * 23.467f) * 18765.4321f, 1.0f);

        if (h1 < 0.0f) h1 += 1.0f;
        if (h2 < 0.0f) h2 += 1.0f;
        if (h3 < 0.0f) h3 += 1.0f;
        if (h4 < 0.0f) h4 += 1.0f;
    }

    p.B = std::max(0.1f, baseB + (h1 - 0.5f) * 2.0f * ampB);
    p.C = std::max(0.1f, baseC + (h2 - 0.5f) * 2.0f * ampC);
    p.D = std::max(0.01f, baseD + (h3 - 0.5f) * 2.0f * ampD);
    p.E = std::max(0.0f, baseE + (h4 - 0.5f) * 2.0f * ampE);

    return p;
}
