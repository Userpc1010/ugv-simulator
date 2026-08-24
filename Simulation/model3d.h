#ifndef MODEL3D_H
#define MODEL3D_H

#include <QVector>
#include <QString>
#include <QMatrix4x4>
#include <QQuaternion>
#include "simpleobject3d.h"
#include "materiallibrary.h"

class QOpenGLShaderProgram;
class QOpenGLFunctions;

class Model3D
{
public:
    // Конструктор принимает путь к OBJ, библиотеку материалов и текстуру по умолчанию
    Model3D(const QString &path, MaterialLibrary *ml, const QImage &defaultImg);
    ~Model3D();

    // Отрисовка всех частей (мешей) модели
    void draw(QOpenGLShaderProgram *program, QOpenGLFunctions *functions);

    // Управление трансформацией всей модели целиком
    void translate(const QVector3D &t) { m_translate += t; }
    void rotate(const QQuaternion &r) { m_rotate = r * m_rotate; }
    void setScale(float s) { m_scale = s; }

    // Геттеры для текущего состояния (опционально)
    QVector3D getTranslate() const { return m_translate; }
    QQuaternion getRotate() const { return m_rotate; }
    float getScale() const { return m_scale; }

private:
    // Внутренний метод для расчета тангенсов (Normal Mapping)
    void calculateTBN(QVector<VertexData> &vertdata);

private:
    QVector<SimpleObject3D*> m_meshes; // Список всех частей модели
    QVector3D m_translate;             // Общее смещение
    QQuaternion m_rotate;              // Общий поворот
    float m_scale = 1.0f;              // Общий масштаб
};

#endif // MODEL3D_H
