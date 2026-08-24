#ifndef SIMPLEOBJECT3D_H
#define SIMPLEOBJECT3D_H

#include<QOpenGLBuffer>
#include<QMatrix4x4>
#include<QVector2D>
#include "materiallibrary.h"

class QOpenGLTexture;
class QOpenGLShaderProgram;
class QOpenGLFunctions;

struct VertexData
{
    VertexData() {}
    VertexData(QVector3D p, QVector2D t, QVector3D n)
        : position(p), textcoord(t), normal(n) {}
    QVector3D position;
    QVector2D textcoord;
    QVector3D normal;
    QVector3D tangent;
    QVector3D bitangent;
};

class SimpleObject3D
{
public:
    SimpleObject3D();
    SimpleObject3D(const QString &path, const QImage &img, MaterialLibrary *ml = nullptr);
    SimpleObject3D(const QVector<VertexData> &vert, const QVector<GLuint> &ind, const QImage &img, int selector ,Material *mat = nullptr);
    ~SimpleObject3D();

    void init(const QVector<VertexData> &vert, const QVector<GLuint> &ind, const QImage &img);
    void init_terrain(const QVector<VertexData> &vert, const QVector<GLuint> &ind, const QImage &img);
    void init_obj(const QVector<VertexData> &vert, const QVector<GLuint> &ind, Material* mat);
    void draw(QOpenGLShaderProgram* program, QOpenGLFunctions* functions);
    void rotate(const QQuaternion &r);
    void rotate_to(const QQuaternion &r);

    void translate(const QVector3D &t);
    void translate_to(const QVector3D &t);
    void scale(const float &s);
    void setGlobalTransform(const QMatrix4x4 &gt);

    QVector3D getTranslate() const { return m_Translate; }
    QQuaternion getRotate() const { return m_Rotate; }
    float getScale() const { return m_Scale; }

private:

    bool loadOBJ(const QString &path, QVector<VertexData> &vertexes, QVector<GLuint> &indexes);

    void calculateTBN(QVector<VertexData> &vertdata);

protected:
    void free();

private:
    QOpenGLBuffer m_VertexBuffer;
    QOpenGLBuffer m_IndexBuffer;
    QOpenGLTexture* m_Texture;
    Material* m_Material;

    QQuaternion m_Rotate;
    QVector3D m_Translate;
    float m_Scale;
    int selectors;
    QMatrix4x4 m_GlobalTransform;
};

#endif // SIMPLEOBJECT3D_H
