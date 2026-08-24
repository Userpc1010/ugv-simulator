#ifndef SKYBOX_H
#define SKYBOX_H

#include "transformational.h"
#include "simpleobject3d.h"
#include <QQuaternion>
#include <QVector3D>
#include <QVector2D>
#include <QMatrix4x4>

class Object3D;
class VertexData;
class QImage;

class SkyBox : public Transformational
{
public:
    SkyBox(float width,
           const QImage &img_forward,
           const QImage &img_top,
           const QImage &img_bottom,
           const QImage &img_left,
           const QImage &img_right,
           const QImage &img_back);
    void fillFace(QVector<VertexData> vertexes, const QImage &img);
    ~SkyBox();
    void draw(QOpenGLShaderProgram* program, QOpenGLFunctions*functions = nullptr);
    void rotate(const QQuaternion &r);
    void translate(const QVector3D &t);
    void scale(const float &s);
    void setGlobalTransform(const QMatrix4x4 &gt);

private:

    QVector<SimpleObject3D*> m_Skybox;
};

#endif // SKYBOX_H
