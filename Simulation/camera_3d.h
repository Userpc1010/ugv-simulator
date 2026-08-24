#ifndef CAMERA_3D_H
#define CAMERA_3D_H

#include <QQuaternion>
#include <QVector3D>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>

class Camera_3D
{
public:
    Camera_3D();

    // Отрисовка (передаёт матрицу вида в шейдер)
    void draw(QOpenGLShaderProgram* program, QOpenGLFunctions* functions = nullptr);

    // Вращение камеры (для свободного режима)
    void rotate_camera(double x, double y);

    // Приближение/отдаление (для свободного режима)
    void camera_zoom(bool zoom);

    // Перемещение (для свободного режима)
    void Front_move();
    void Back_move();
    void right_move();
    void left_move();
    void Up_move();
    void Down_move();

    // Явная установка позиции и ориентации (для следящего режима)
    void setPosition(const QVector3D& pos);
    void setYaw(double yawDeg);
    void setPitch(double pitchDeg);
    void lookAt(const QVector3D& target);  // направляет камеру на цель

    // Геттеры
    QVector3D getPosition() const { return m_Translate; }
    QVector3D getFront()    const { return front; }
    double    getYaw()       const { return yaw; }
    double    getPitch()     const { return pitch; }

private:
    void updateViewMatrix();  // пересчитывает m_ViewMatrix из текущих параметров

private:
    QVector3D m_Translate;      // позиция камеры в мире
    QVector3D m_camera_up;      // вектор "вверх" (обычно (0,1,0))
    QVector3D front;             // направление взгляда

    QMatrix4x4 m_GlobalTransform; // глобальная трансформация (обычно единичная)
    QMatrix4x4 m_ViewMatrix;      // итоговая матрица вида

    double pitch;                // угол наклона (в градусах)
    double yaw;                  // угол поворота (в градусах)

    double sensitivity_x;         // чувствительность мыши по горизонтали
    double sensitivity_y;         // чувствительность мыши по вертикали
};

#endif // CAMERA_3D_H
