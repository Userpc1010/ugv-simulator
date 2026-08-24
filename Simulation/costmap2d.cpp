#include "costmap2d.h"
#include <QOpenGLTexture>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>

CostmapRander::CostmapRander()
    : m_VertexBuffer(QOpenGLBuffer::VertexBuffer)
    , m_IndexBuffer(QOpenGLBuffer::IndexBuffer)
    , m_Texture(nullptr)
{
}

CostmapRander::~CostmapRander()
{
    free();
}

void CostmapRander::init(float worldWidth, float worldHeight, float yLevel)
{
    free();

    float hw = worldWidth * 0.5f;
    float hh = worldHeight * 0.5f;

    GLfloat vertices[] = {
        // position.x, position.y, position.z, texcoord.x, texcoord.y
         hw, yLevel, -hh,  1.0f, 0.0f,  // 0: дальний правый
        -hw, yLevel, -hh,  0.0f, 0.0f,  // 1: дальний левый
         hw, yLevel,  hh,  1.0f, 1.0f,  // 2: ближний правый
        -hw, yLevel,  hh,  0.0f, 1.0f   // 3: ближний левый
    };

    GLubyte indices[] = {0, 1, 2, 3};

    m_VertexBuffer.create();
    m_VertexBuffer.bind();
    m_VertexBuffer.allocate(vertices, sizeof(vertices));
    m_VertexBuffer.release();

    m_IndexBuffer.create();
    m_IndexBuffer.bind();
    m_IndexBuffer.allocate(indices, sizeof(indices));
    m_IndexBuffer.release();
}

void CostmapRander::draw(QOpenGLShaderProgram* program, QOpenGLFunctions* functions)
{
    if (!m_VertexBuffer.isCreated() || !m_IndexBuffer.isCreated()) return;
    if (!m_Texture || !m_Texture->isCreated()) return;

    QMatrix4x4 identityMatrix;
    identityMatrix.setToIdentity();
    program->setUniformValue("u_modelMatrix", identityMatrix);

    program->setUniformValue("r_id", 0);
    program->setUniformValue("p_id", 0);
    m_Texture->bind(0);
    program->setUniformValue("u_texture", 0);

    m_VertexBuffer.bind();
    m_IndexBuffer.bind();

    int vertloc = program->attributeLocation("a_position");
    program->enableAttributeArray(vertloc);
    program->setAttributeBuffer(vertloc, GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));

    int texloc = program->attributeLocation("a_textcoord");
    program->enableAttributeArray(texloc);
    program->setAttributeBuffer(texloc, GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));

    functions->glDrawElements(GL_TRIANGLE_STRIP, 4, GL_UNSIGNED_BYTE, nullptr);

    m_VertexBuffer.release();
    m_IndexBuffer.release();
    m_Texture->release();
}

void CostmapRander::update_map(const QImage& img)
{
    if (m_Texture && m_Texture->isCreated()) {
        delete m_Texture;
        m_Texture = nullptr;
    }

    m_Texture = new QOpenGLTexture(img);
    m_Texture->setMinificationFilter(QOpenGLTexture::Nearest);
    m_Texture->setMagnificationFilter(QOpenGLTexture::Nearest);
    m_Texture->setWrapMode(QOpenGLTexture::ClampToEdge);
}

void CostmapRander::clear()
{
    if (m_Texture && m_Texture->isCreated()) {
        delete m_Texture;
        m_Texture = nullptr;
    }
}

void CostmapRander::free()
{
    if (m_VertexBuffer.isCreated()) m_VertexBuffer.destroy();
    if (m_IndexBuffer.isCreated()) m_IndexBuffer.destroy();
    if (m_Texture && m_Texture->isCreated()) {
        delete m_Texture;
        m_Texture = nullptr;
    }
}
