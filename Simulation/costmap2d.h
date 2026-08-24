#ifndef COSTMAP2D_H
#define COSTMAP2D_H

#include <QOpenGLBuffer>
#include <QOpenGLTexture>

class QOpenGLShaderProgram;
class QOpenGLFunctions;

class CostmapRander
{
public:
    CostmapRander();
    ~CostmapRander();

    void init(float worldWidth, float worldHeight, float yLevel);
    void draw(QOpenGLShaderProgram* program, QOpenGLFunctions* functions);
    void update_map(const QImage &img);
    void clear();

private:
    void free();

    QOpenGLBuffer m_VertexBuffer;
    QOpenGLBuffer m_IndexBuffer;
    QOpenGLTexture* m_Texture;
};

#endif // COSTMAP2D_H
