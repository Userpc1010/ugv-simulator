#include "plot.h"
#include "ui_plot.h"


Plot::Plot(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Plot)
{
    ui->setupUi(this);
    ui->centralWidget->setGeometry(640, 480, 640, 480);
    ui->centralWidget->move(0,0);

    setupQuadraticDemo(ui->customPlot);

    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    setWindowTitle("Output: ");
    ui->customPlot->replot();
}

Plot::~Plot()
{
    delete ui;
}

void Plot::setupQuadraticDemo(QCustomPlot *customPlot)
{

  // configure axis rect:
  customPlot->plotLayout()->clear(); // clear default axis rect so we can start from scratch

  QCPAxisRect *wideAxis_1 = new QCPAxisRect(customPlot);
  QCPAxisRect *wideAxis_2 = new QCPAxisRect(customPlot);
  QCPAxisRect *wideAxis_3 = new QCPAxisRect(customPlot);

  wideAxis_1->setupFullAxesBox(true);
  wideAxis_2->setupFullAxesBox(true);
  wideAxis_3->setupFullAxesBox(true);

  customPlot->plotLayout()->addElement(0, 0, wideAxis_1);
  customPlot->plotLayout()->addElement(1, 0, wideAxis_2);
  customPlot->plotLayout()->addElement(2, 0, wideAxis_3);


  // create graph and assign data to it:
  customPlot->addGraph(wideAxis_1->axis(QCPAxis::atBottom), wideAxis_1->axis(QCPAxis::atLeft));
  customPlot->graph(0)->setPen(QPen(QColor(0, 0, 200)));
  customPlot->graph(0)->setName("X");

  // create graph and assign data to it:
  customPlot->addGraph(wideAxis_2->axis(QCPAxis::atBottom), wideAxis_2->axis(QCPAxis::atLeft));
  customPlot->graph(1)->setPen(QPen(QColor(200, 0, 0)));
  customPlot->graph(1)->setName("Y");

  // create graph and assign data to it:
  customPlot->addGraph(wideAxis_3->axis(QCPAxis::atBottom), wideAxis_3->axis(QCPAxis::atLeft));
  customPlot->graph(2)->setPen(QPen(QColor(0, 200, 0)));
  customPlot->graph(2)->setName("Z");

}

void Plot::Draw(double x, double y, double z, double dt)
{ i++;

  ui->customPlot->graph(0)->addData(i, x);
  ui->customPlot->graph(0)->rescaleAxes();
  ui->customPlot->graph(1)->addData(i, y);
  ui->customPlot->graph(1)->rescaleAxes();
  ui->customPlot->graph(2)->addData(i, z);
  ui->customPlot->graph(2)->rescaleAxes();

  if(i >= 20000) {i = 0;
      ui->customPlot->graph(0)->data()->clear();
      ui->customPlot->graph(1)->data()->clear();
      ui->customPlot->graph(2)->data()->clear();
  }

  ui->customPlot->replot();
}

void Plot::Draw( Eigen::Vector3d r, double dt)
{
   i++;

   ui->customPlot->graph(0)->addData(i, r.x());
   ui->customPlot->graph(0)->rescaleAxes();
   ui->customPlot->graph(1)->addData(i, r.y());
   ui->customPlot->graph(1)->rescaleAxes();
   ui->customPlot->graph(2)->addData(i, r.z());
   ui->customPlot->graph(2)->rescaleAxes();

   if(i >= 20000) {i = 0;
     ui->customPlot->graph(0)->data()->clear();
     ui->customPlot->graph(1)->data()->clear();
     ui->customPlot->graph(2)->data()->clear();
   }

   ui->customPlot->replot();
}

