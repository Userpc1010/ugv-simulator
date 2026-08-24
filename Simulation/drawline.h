#ifndef DRAWLINE_H
#define DRAWLINE_H

#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>
#include <QVector3D>

class DrawLine
{
public:
    DrawLine();
    ~DrawLine();

    void DrawLines(GLfloat* vertices_buffer, QVector3D color_buffer,
                   unsigned long long counter);
    void draw(QOpenGLShaderProgram* program, QOpenGLFunctions* functions);
    void clear();

private:
    QOpenGLBuffer m_VertexBuffer;
    QVector3D m_color;
    unsigned long long m_vertexCount;
    bool m_hasData;
};

#endif // DRAWLINE_H
