#include "oglwidget.h"
#include <QApplication>
#include "plot.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QSurfaceFormat format;
    format.setSamples(16);
    format.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(format);

    //Plot plot;
    OGLWidget w;

    //QObject::connect(&w, SIGNAL(Draw(imu::Vector<3> ,double)), &plot, SLOT(Draw(imu::Vector<3> ,double)));

    //plot.show();
    w.show();

    return a.exec();
}
