#include "model3d.h"
#include <QFileInfo>
#include <QDir>
#include "material.h"

Model3D::Model3D(const QString &path, MaterialLibrary *ml, const QImage &defaultImg)
    : m_scale(1.0f)
{
    // Проверка на пустоту (аналог m_OBJ_Objects.size())
    if(!m_meshes.isEmpty()) { qCritical() << "EngineObject not empty!"; return; }

    QFile objfile(path);
    if(!objfile.exists()) { qCritical() << "File not exist:" << path; return; }
    if(!objfile.open(QFile::ReadOnly)) { qCritical() << "File not opened:" << path; return; }

    QTextStream input(&objfile);
    QVector<QVector3D> coords;
    QVector<QVector2D> texturcoords;
    QVector<QVector3D> normals;

    QVector<VertexData> vertexes;
    QVector<GLuint> indexes;
    QString mtlName;

    qDebug() << "Reading" << path << "...";

    bool ok = true;
    while(!input.atEnd() && ok)
    {
        auto str = input.readLine(); if(str.isEmpty()) continue;
        auto strlist = str.split(' ', QString::SkipEmptyParts);
        if(strlist.isEmpty()) continue;
        auto key = strlist.at(0).toLower();

        if (key == "#") { qDebug() << str; }
               else if(key == "mtllib") {
                   if(strlist.size() > 1) {
                       auto file = QFileInfo(path).absolutePath() + QDir::separator() + strlist.at(1);
                       if(QFile(file).exists()) ok = ml->load(file);
                       else { qCritical() << "File not exists:" << file; ok = false; }
                   }
                   else { qCritical() << "Error at line (count):" << str; ok = false; }
               }
               else if(key == "v") {
                   if(strlist.size() > 3) {
                       coords.append(QVector3D(strlist.at(1).toFloat(&ok), strlist.at(2).toFloat(&ok), strlist.at(3).toFloat(&ok)));
                       if(!ok) { qCritical() << "Error at line (format):" << str; }
                   }
                   else { qCritical() << "Error at line (count):" << str; ok = false; }
               }
               else if(key == "vt") {
                   if(strlist.size() > 2) {
                       texturcoords.append(QVector2D(strlist.at(1).toFloat(&ok), strlist.at(2).toFloat(&ok)));
                       if(!ok) { qCritical() << "Error at line (format):" << str; }
                   }
                   else { qCritical() << "Error at line (count):" << str; ok = false; }
               }
               else if(key == "vn") {
                   if(strlist.size() == 4) {
                       normals.append(QVector3D(strlist.at(1).toFloat(&ok), strlist.at(2).toFloat(&ok), strlist.at(3).toFloat(&ok)));
                       if(!ok) { qCritical() << "Error at line (format):" << str; }
                   }
                   else { qCritical() << "Error at line (count):" << str; ok = false; }
               }
        else if(key == "f")
        {
            QVector<GLuint> faceIndices;

            for(int i = 1; i < strlist.size(); i++)
            {
                auto v = strlist.at(i).split('/');

                // Формат: только вершины (v)
                if(v.size() == 1) {
                    int idx = v[0].toInt(&ok, 10) - 1;
                    if(!ok || idx < 0 || idx >= coords.size()) {
                        qCritical() << "Invalid vertex index:" << strlist.at(i);
                        ok = false; break;
                    }

                    QVector2D texCoord(0,0);
                    QVector3D normal(0,1,0);
                    if(!texturcoords.isEmpty()) texCoord = texturcoords.at(qMin(idx, texturcoords.size() - 1));
                    if(!normals.isEmpty()) normal = normals.at(qMin(idx, normals.size() - 1));

                    vertexes.append(VertexData(coords.at(idx), texCoord, normal));
                    faceIndices.append(vertexes.size() - 1);
                }
                // Формат: вершина и текстура (v/vt)
                else if(v.size() == 2) {
                    int vIdx = v[0].toInt(&ok, 10) - 1;
                    int vtIdx = v[1].toInt(&ok, 10) - 1;

                    if(!ok || vIdx < 0 || vIdx >= coords.size() || vtIdx < 0 || vtIdx >= texturcoords.size()) {
                        qCritical() << "Invalid vertex/uv index:" << strlist.at(i);
                        ok = false; break;
                    }

                    QVector3D normal(0,1,0);
                    if(!normals.isEmpty()) normal = normals.at(qMin(vIdx, normals.size() - 1));

                    vertexes.append(VertexData(coords.at(vIdx), texturcoords.at(vtIdx), normal));
                    faceIndices.append(vertexes.size() - 1);
                }
                // Формат: вершина/текстура/нормаль (v/vt/vn) или вершина//нормаль (v//vn)
                else if(v.size() >= 3) {
                    int vIdx = v[0].toInt(&ok, 10) - 1;
                    int vtIdx = v[1].isEmpty() ? 0 : v[1].toInt(&ok, 10) - 1;
                    int vnIdx = v[2].toInt(&ok, 10) - 1;

                    if(!ok || vIdx < 0 || vIdx >= coords.size() || vnIdx < 0 || vnIdx >= normals.size()) {
                        qCritical() << "Invalid vertex/uv/normal index:" << strlist.at(i);
                        ok = false; break;
                    }

                    QVector2D texCoord(0,0);
                    if(!texturcoords.isEmpty() && vtIdx >= 0 && vtIdx < texturcoords.size()) texCoord = texturcoords.at(vtIdx);

                    vertexes.append(VertexData(coords.at(vIdx), texCoord, normals.at(vnIdx)));
                    faceIndices.append(vertexes.size() - 1);
                }
            }

            // Триангуляция
            if(faceIndices.size() >= 3 && ok) {
                for(int j = 0; j < faceIndices.size() - 2; j++) {
                    indexes.append(faceIndices[0]);
                    indexes.append(faceIndices[j + 1]);
                    indexes.append(faceIndices[j + 2]);
                }
            }
            if(!ok) { qCritical() << "Error at line (format):" << str; }
        }
        else if(key == "usemtl")
        {
            if(!vertexes.isEmpty()) {
                calculateTBN(vertexes);
                Material* m = ml ? ml->get(mtlName) : nullptr;
                QImage img = (m && m->isUseDiffuseMap()) ? m->DiffuseMap() : defaultImg;
                m_meshes.append(new SimpleObject3D(vertexes, indexes, img, 1, m));
                vertexes.clear(); indexes.clear();
            }

            if(strlist.size() > 1) {
                   mtlName = strlist.at(1);
               } else {
                   qCritical() << "Error at line (material name missing):" << str;
                   ok = false;
               }


        }
    }

    if(!vertexes.isEmpty()) { // Последний объект
        calculateTBN(vertexes);
        Material* m = ml ? ml->get(mtlName) : nullptr;
        QImage img = (m && m->isUseDiffuseMap()) ? m->DiffuseMap() : defaultImg;
        m_meshes.append(new SimpleObject3D(vertexes, indexes, img, 1, m));
    }
    objfile.close();
}

Model3D::~Model3D() { qDeleteAll(m_meshes); }


void Model3D::draw(QOpenGLShaderProgram *program, QOpenGLFunctions *functions) {
    QMatrix4x4 rootMatrix;
    rootMatrix.translate(m_translate);
    rootMatrix.rotate(m_rotate);
    rootMatrix.scale(m_scale);

    for (auto *mesh : m_meshes) {
        mesh->setGlobalTransform(rootMatrix);
        mesh->draw(program, functions);
    }
}

void Model3D::calculateTBN(QVector<VertexData> &vertdata)
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
