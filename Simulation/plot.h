#ifndef PLOT_H
#define PLOT_H

#include <QWidget>
#include <QVector>
#include <Eigen/Dense>
#include "qcustomplot.h"

namespace Ui {
class Plot;
}

class Plot : public QWidget
{
    Q_OBJECT

public:
    explicit Plot(QWidget *parent = nullptr);
    ~Plot();

    void setupQuadraticDemo(QCustomPlot *customPlot);

public slots:

void Draw (double x, double y, double z, double dt);

void Draw (Eigen::Vector3d r, double dt);

private:
    Ui::Plot *ui;

    int i = 0;
};

#endif // PLOT_H
