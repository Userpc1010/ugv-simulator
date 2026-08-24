#include "simpleobject3d.h"
#include<QOpenGLTexture>
#include<QOpenGLShaderProgram>
#include<QOpenGLFunctions>
#include<QFile>

SimpleObject3D::SimpleObject3D()
    : m_VertexBuffer(QOpenGLBuffer::VertexBuffer), // Инициализация буфера вершин
      m_IndexBuffer(QOpenGLBuffer::IndexBuffer),   // Инициализация буфера индексов
      m_Texture(nullptr),
      m_Material(nullptr),
      m_Scale(1.0f)
{
    m_Scale = 1.0f;
}

SimpleObject3D::SimpleObject3D(const QString &path, const QImage &img, MaterialLibrary *ml)
    : m_VertexBuffer(QOpenGLBuffer::VertexBuffer), // Инициализация буфера вершин
      m_IndexBuffer(QOpenGLBuffer::IndexBuffer),   // Инициализация буфера индексов
      m_Texture(nullptr),
      m_Material(nullptr),
      m_Scale(1.0f)
{
    QVector<VertexData> vertexes;
       QVector<GLuint> indexes;

       if (loadOBJ(path, vertexes, indexes)) {
           calculateTBN(vertexes);
           init(vertexes, indexes, img);
           qDebug() << "Object loaded from" << path;
       } else {
           qCritical() << "Failed to load OBJ from" << path;
       }
}

SimpleObject3D::SimpleObject3D(const QVector<VertexData> &vert, const QVector<GLuint> &ind, const QImage &img, int selector ,Material *mat)
    : m_VertexBuffer(QOpenGLBuffer::VertexBuffer), // Инициализация буфера вершин
      m_IndexBuffer(QOpenGLBuffer::IndexBuffer),   // Инициализация буфера индексов
      m_Texture(nullptr),
      m_Material(nullptr),
      m_Scale(1.0f)
{
    m_Scale = 1.0f;

    switch (selector) {

    case 0: init(vert, ind, img); break;
    case 1: init_obj(vert, ind, mat); break;
    case 2: init_terrain(vert, ind, img); break;

    default: init(vert, ind, img); break;
    }
}



SimpleObject3D::~SimpleObject3D()
{
    free();
}

void SimpleObject3D::free()
{
    if(m_VertexBuffer.isCreated()) m_VertexBuffer.destroy();
    if(m_IndexBuffer.isCreated()) m_IndexBuffer.destroy();
    if(m_Texture != nullptr && m_Texture->isCreated())
    { delete m_Texture; m_Texture = nullptr; }
}

void SimpleObject3D::init(const QVector<VertexData> &vert, const QVector<GLuint> &ind, const QImage &img)
{
    free();

    m_VertexBuffer.create();
    m_VertexBuffer.bind();
    m_VertexBuffer.allocate(vert.constData(), vert.size() * static_cast<int>(sizeof(VertexData)));
    m_VertexBuffer.release();// temp

    m_IndexBuffer.create();
    m_IndexBuffer.bind();
    m_IndexBuffer.allocate(ind.constData(), ind.size() * static_cast<int>(sizeof(GLuint)));
    m_IndexBuffer.release();// temp

    m_Texture = new QOpenGLTexture(img.mirrored());
    m_Texture->setMinificationFilter(QOpenGLTexture::Nearest);
    m_Texture->setMagnificationFilter(QOpenGLTexture::Linear);
    m_Texture->setWrapMode(QOpenGLTexture::Repeat);
}

void SimpleObject3D::init_terrain(const QVector<VertexData> &vert, const QVector<GLuint> &ind, const QImage &img)
{
    free();

    m_VertexBuffer.create();
    m_VertexBuffer.bind();
    m_VertexBuffer.allocate(vert.constData(), vert.size() * static_cast<int>(sizeof(VertexData)));
    m_VertexBuffer.release();// temp

    m_IndexBuffer.create();
    m_IndexBuffer.bind();
    m_IndexBuffer.allocate(ind.constData(), ind.size() * static_cast<int>(sizeof(GLuint)));
    m_IndexBuffer.release();// temp

    m_Texture = new QOpenGLTexture(img.mirrored());
    m_Texture->setMinificationFilter(QOpenGLTexture::Nearest);
    m_Texture->setMagnificationFilter(QOpenGLTexture::Linear);
    m_Texture->setWrapMode(QOpenGLTexture::Repeat);
}

void SimpleObject3D::init_obj(const QVector<VertexData> &vert, const QVector<GLuint> &ind, Material *mat)
{
    free();

    m_VertexBuffer.create();
    m_VertexBuffer.bind();
    m_VertexBuffer.allocate(vert.constData(), vert.size() * static_cast<int>(sizeof(VertexData)));
    m_VertexBuffer.release();

    m_IndexBuffer.create();
    m_IndexBuffer.bind();
    m_IndexBuffer.allocate(ind.constData(), ind.size() * static_cast<int>(sizeof(GLuint)));
    m_IndexBuffer.release();

    m_Material = mat;

//    if(m_Material->isUseDiffuseMap())
//    {
//        m_DiffuseMap = new QOpenGLTexture(m_Material->DiffuseMap().mirrored());
//        m_DiffuseMap->setMinificationFilter(QOpenGLTexture::Nearest);
//        m_DiffuseMap->setMagnificationFilter(QOpenGLTexture::Linear);
//        m_DiffuseMap->setWrapMode(QOpenGLTexture::Repeat);
//    }

//    if(m_Material->isUseNormalMap())
//    {
//        m_NormalMap = new QOpenGLTexture(m_Material->NormalMap().mirrored());
//        m_NormalMap->setMinificationFilter(QOpenGLTexture::Nearest);
//        m_NormalMap->setMagnificationFilter(QOpenGLTexture::Linear);
//        m_NormalMap->setWrapMode(QOpenGLTexture::Repeat);
//    }
}

void SimpleObject3D::draw(QOpenGLShaderProgram *program, QOpenGLFunctions *functions)
{
    if(!m_VertexBuffer.isCreated()) {qCritical() << "Vertex buffer is not created"; return; }
    if(!m_IndexBuffer.isCreated()) {qCritical() << "Index buffer is not created"; return; }


    QMatrix4x4 modelMatrix;
    modelMatrix.setToIdentity();
    modelMatrix.translate(m_Translate); // здесь важен порядок преобразований *
    modelMatrix.rotate(m_Rotate); // *
    modelMatrix.scale(m_Scale); // *
    modelMatrix = m_GlobalTransform * modelMatrix;

    m_Texture->bind(0);
    program->setUniformValue("u_texture", 0);
    program->setUniformValue("u_modelMatrix", modelMatrix);

    m_VertexBuffer.bind();

    int offset = 0;

    int vertloc = program->attributeLocation("a_position");
    program->enableAttributeArray(vertloc);
    program->setAttributeBuffer(vertloc, GL_FLOAT, offset, 3, sizeof(VertexData));

    offset += sizeof(QVector3D);

    int texloc = program->attributeLocation("a_textcoord");
    program->enableAttributeArray(texloc);
    program->setAttributeBuffer(texloc, GL_FLOAT, offset, 2, sizeof(VertexData));

    offset += sizeof(QVector2D);

    int normloc = program->attributeLocation("a_normal");
    program->enableAttributeArray(normloc);
    program->setAttributeBuffer(normloc, GL_FLOAT, offset, 3, sizeof(VertexData));

    m_IndexBuffer.bind();

    functions->glDrawElements(GL_TRIANGLES,(m_IndexBuffer.size()/4), GL_UNSIGNED_INT, nullptr);

    m_VertexBuffer.release();
    m_IndexBuffer.release();
    m_Texture->release();
}

void SimpleObject3D::rotate(const QQuaternion &r)
{
    m_Rotate = r * m_Rotate;
}

void SimpleObject3D::rotate_to(const QQuaternion &r)
{
  m_Rotate = r;
}

void SimpleObject3D::translate(const QVector3D &t)
{
    m_Translate += t;
}

void SimpleObject3D::translate_to(const QVector3D &t)
{
   m_Translate = t;
}

void SimpleObject3D::scale(const float &s)
{
    m_Scale *= s;
}

void SimpleObject3D::setGlobalTransform(const QMatrix4x4 &gt)
{
    m_GlobalTransform = gt;
}

bool SimpleObject3D::loadOBJ(const QString &path, QVector<VertexData> &vertexes, QVector<GLuint> &indexes)
{
    QFile objfile(path);
    if (!objfile.open(QFile::ReadOnly)) return false;

    QTextStream input(&objfile);
    QVector<QVector3D> coords;
    QVector<QVector2D> texturcoords;
    QVector<QVector3D> normals;

    bool ok = true;
    while (!input.atEnd() && ok) {
        QString str = input.readLine();
        QStringList strlist = str.split(' ', QString::SkipEmptyParts);
        if (strlist.isEmpty()) continue;

        QString key = strlist.at(0).toLower();

        if (key == "v") {
            coords.append(QVector3D(strlist.at(1).toFloat(&ok), strlist.at(2).toFloat(&ok), strlist.at(3).toFloat(&ok)));
        } else if (key == "vt") {
            texturcoords.append(QVector2D(strlist.at(1).toFloat(&ok), strlist.at(2).toFloat(&ok)));
        } else if (key == "vn") {
            normals.append(QVector3D(strlist.at(1).toFloat(&ok), strlist.at(2).toFloat(&ok), strlist.at(3).toFloat(&ok)));
        } else if (key == "f") {
            QVector<GLuint> faceIndices;
            for (int i = 1; i < strlist.size(); i++) {
                QStringList v = strlist.at(i).split('/');

                int vIdx = v[0].toInt(&ok) - 1;
                int vtIdx = (v.size() > 1 && !v[1].isEmpty()) ? v[1].toInt(&ok) - 1 : -1;
                int vnIdx = (v.size() > 2 && !v[2].isEmpty()) ? v[2].toInt(&ok) - 1 : -1;

                if (!ok || vIdx < 0 || vIdx >= coords.size()) break;

                QVector2D uv = (vtIdx >= 0 && vtIdx < texturcoords.size()) ? texturcoords.at(vtIdx) : QVector2D(0, 0);
                QVector3D norm = (vnIdx >= 0 && vnIdx < normals.size()) ? normals.at(vnIdx) : QVector3D(0, 1, 0);

                vertexes.append(VertexData(coords.at(vIdx), uv, norm));
                faceIndices.append(vertexes.size() - 1);
            }

            // Триангуляция (Fan Triangulation)
            if (faceIndices.size() >= 3) {
                for (int j = 0; j < faceIndices.size() - 2; j++) {
                    indexes.append(faceIndices[0]);
                    indexes.append(faceIndices[j + 1]);
                    indexes.append(faceIndices[j + 2]);
                }
            }
        }
    }
    objfile.close();
    return ok;
}

void SimpleObject3D::calculateTBN(QVector<VertexData> &vertdata)
{
    for(int i = 0; i < vertdata.size(); i += 3)
    {
        auto v1 = vertdata.at(i).position;
        auto v2 = vertdata.at(i + 1).position;
        auto v3 = vertdata.at(i + 2).position;

        auto uv1 = vertdata.at(i).textcoord;
        auto uv2 = vertdata.at(i + 1).textcoord;
        auto uv3 = vertdata.at(i + 2).textcoord;

        auto deltaPos1 = v2 - v1;
        auto deltaPos2 = v3 - v1;
        auto deltaUV1 = uv2 - uv1;
        auto deltaUV2 = uv3 - uv1;

        float r = 1.0f / (deltaUV1.x() * deltaUV2.y() - deltaUV1.y() * deltaUV2.x());
        QVector3D tangent = (deltaPos1 * deltaUV2.y() - deltaPos2 * deltaUV1.y()) * r;
        QVector3D bitangent = (deltaPos2 * deltaUV1.x() - deltaPos1 * deltaUV2.x()) * r;

        vertdata[i].tangent = tangent;
        vertdata[i + 1].tangent = tangent;
        vertdata[i + 2].tangent = tangent;

        vertdata[i].bitangent = bitangent;
        vertdata[i + 1].bitangent = bitangent;
        vertdata[i + 2].bitangent = bitangent;
    }
}
