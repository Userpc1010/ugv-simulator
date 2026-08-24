#include "multidrawline.h"

MultiDrawLine::MultiDrawLine() {}

MultiDrawLine::~MultiDrawLine()
{
    Clear_Lines();
}

void MultiDrawLine::draw_lines(QOpenGLShaderProgram* program, QOpenGLFunctions* functions)
{
    for (auto o : m_Objects_Line) {
        if (o != nullptr) {
            o->draw(program, functions);
        }
    }
}

void MultiDrawLine::Add_Lines(GLfloat* vertices_buffer, QVector3D color_buffer,
                               unsigned long long counter, uint16_t index)
{
    // Убедимся, что массив достаточно большой
    while (m_Objects_Line.size() < index) {
        m_Objects_Line.append(nullptr);
    }

    if (m_Objects_Line[index - 1] == nullptr) {
        m_Objects_Line[index - 1] = new DrawLine();
    }

    m_Objects_Line[index - 1]->DrawLines(vertices_buffer, color_buffer, counter);
}

void MultiDrawLine::Remove_Lines(uint16_t index)
{
    if (index > 0 && index <= m_Objects_Line.size()) {
        delete m_Objects_Line[index - 1];
        m_Objects_Line[index - 1] = nullptr;
    }
}

void MultiDrawLine::Clear_Lines()
{
    for (auto o : m_Objects_Line) {
        delete o;
    }
    m_Objects_Line.clear();
}
