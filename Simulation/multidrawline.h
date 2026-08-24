#ifndef MULTIDRAWLINE_H
#define MULTIDRAWLINE_H

#include "drawline.h"
#include <QVector>

class MultiDrawLine
{
public:
    MultiDrawLine();
    ~MultiDrawLine();

    void draw_lines(QOpenGLShaderProgram* program, QOpenGLFunctions* functions);
    void Add_Lines(GLfloat* vertices_buffer, QVector3D color_buffer,
                   unsigned long long counter, uint16_t index);
    void Remove_Lines(uint16_t index);
    void Clear_Lines();

private:
    QVector<DrawLine*> m_Objects_Line;
};

#endif // MULTIDRAWLINE_H
