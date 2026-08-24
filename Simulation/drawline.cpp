#include "drawline.h"

DrawLine::DrawLine()
    : m_VertexBuffer(QOpenGLBuffer::VertexBuffer)
    , m_vertexCount(0)
    , m_hasData(false)
{
}

DrawLine::~DrawLine()
{
    clear();
}

void DrawLine::DrawLines(GLfloat* vertices_buffer, QVector3D color_buffer,
                          unsigned long long counter)
{
    clear();

    m_color = color_buffer;
    m_vertexCount = counter;

    m_VertexBuffer.create();
    m_VertexBuffer.bind();
    m_VertexBuffer.allocate(vertices_buffer, counter * 3 * sizeof(GLfloat));
    m_VertexBuffer.release();

    m_hasData = true;
}

void DrawLine::draw(QOpenGLShaderProgram* program, QOpenGLFunctions* functions)
{
    if (!m_hasData || !m_VertexBuffer.isCreated()) return;

    QMatrix4x4 identityMatrix;
    identityMatrix.setToIdentity();
    program->setUniformValue("u_modelMatrix", identityMatrix);

    program->setUniformValue("r_id", 2);
    program->setUniformValue("p_id", 2);
    program->setUniformValue("u_color", m_color);

    m_VertexBuffer.bind();

    int vertloc = program->attributeLocation("a_position");
    program->enableAttributeArray(vertloc);
    program->setAttributeBuffer(vertloc, GL_FLOAT, 0, 3, 0);

    functions->glDrawArrays(GL_LINES, 0, m_vertexCount);

    m_VertexBuffer.release();
}

void DrawLine::clear()
{
    if (m_VertexBuffer.isCreated()) {
        m_VertexBuffer.destroy();
    }
    m_hasData = false;
    m_vertexCount = 0;
}
