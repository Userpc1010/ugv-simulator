#include "camera_3d.h"
#include <QtMath>

Camera_3D::Camera_3D()
    : m_Translate(0.0f, 100.0f, 200.0f),   // начальная позиция над ландшафтом
      m_camera_up(0.0f, 1.0f, 0.0f),
      pitch(-30.0),                         // смотрим немного вниз
      yaw(-90.0),                            // смотрим вдоль оси -Z
      sensitivity_x(0.5),
      sensitivity_y(0.5)
{
    // Рассчитываем начальный вектор front
    double yawRad = qDegreesToRadians(yaw);
    double pitchRad = qDegreesToRadians(pitch);
    front.setX(cos(yawRad) * cos(pitchRad));
    front.setY(sin(pitchRad));
    front.setZ(sin(yawRad) * cos(pitchRad));
    front.normalize();

    updateViewMatrix();
}

void Camera_3D::draw(QOpenGLShaderProgram *program, QOpenGLFunctions *functions)
{
    Q_UNUSED(functions); // параметр не используется
    updateViewMatrix();
    program->setUniformValue("u_viewMatrix", m_ViewMatrix);
}

void Camera_3D::rotate_camera(double x, double y)
{
    yaw += x * sensitivity_x;
    pitch += y * sensitivity_y;

    // Ограничиваем pitch, чтобы избежать опрокидывания
    if (pitch > 89.9)  pitch = 89.9;
    if (pitch < -89.9) pitch = -89.9;

    double yawRad = qDegreesToRadians(yaw);
    double pitchRad = qDegreesToRadians(pitch);

    front.setX(cos(yawRad) * cos(pitchRad));
    front.setY(sin(pitchRad));
    front.setZ(sin(yawRad) * cos(pitchRad));
    front.normalize();
}

void Camera_3D::camera_zoom(bool zoom)
{
    if (zoom)
        m_Translate += front * 10.0f;   // приближение
    else
        m_Translate -= front * 10.0f;   // отдаление
}

void Camera_3D::Front_move()
{
    m_Translate += front * 10.0f;
}

void Camera_3D::Back_move()
{
    m_Translate -= front * 10.0f;
}

void Camera_3D::right_move()
{
    // Вектор вправо = вектор вперёд × вектор вверх
    QVector3D right = QVector3D::crossProduct(front, m_camera_up).normalized();
    m_Translate += right * 10.0f;
}

void Camera_3D::left_move()
{
    QVector3D right = QVector3D::crossProduct(front, m_camera_up).normalized();
    m_Translate -= right * 10.0f;
}

void Camera_3D::Up_move()
{
    m_Translate += m_camera_up * 10.0f;
}

void Camera_3D::Down_move()
{
    m_Translate -= m_camera_up * 10.0f;
}

void Camera_3D::setPosition(const QVector3D &pos)
{
    m_Translate = pos;
}

void Camera_3D::setYaw(double yawDeg)
{
    yaw = yawDeg;
    // Пересчитываем front, чтобы он соответствовал новым углам
    double yawRad = qDegreesToRadians(yaw);
    double pitchRad = qDegreesToRadians(pitch);
    front.setX(cos(yawRad) * cos(pitchRad));
    front.setY(sin(pitchRad));
    front.setZ(sin(yawRad) * cos(pitchRad));
    front.normalize();
}

void Camera_3D::setPitch(double pitchDeg)
{
    pitch = pitchDeg;
    if (pitch > 89.9)  pitch = 89.9;
    if (pitch < -89.9) pitch = -89.9;
    double yawRad = qDegreesToRadians(yaw);
    double pitchRad = qDegreesToRadians(pitch);
    front.setX(cos(yawRad) * cos(pitchRad));
    front.setY(sin(pitchRad));
    front.setZ(sin(yawRad) * cos(pitchRad));
    front.normalize();
}

void Camera_3D::lookAt(const QVector3D &target)
{
    // Вычисляем направление от камеры к цели
    QVector3D direction = target - m_Translate;
    front = direction.normalized();

    // Вычисляем углы yaw и pitch из вектора front
    // pitch = asin(front.y())
    pitch = qRadiansToDegrees(asin(front.y()));
    // yaw = atan2(front.z(), front.x())
    yaw = qRadiansToDegrees(atan2(front.z(), front.x()));
}

void Camera_3D::updateViewMatrix()
{
    m_ViewMatrix.setToIdentity();
    m_ViewMatrix.lookAt(m_Translate,
                        m_Translate + front,
                        m_camera_up);
    // Если нужна глобальная трансформация, применяем её
    m_ViewMatrix = m_ViewMatrix * m_GlobalTransform.inverted();
}
