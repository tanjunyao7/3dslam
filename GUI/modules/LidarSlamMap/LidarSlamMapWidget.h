#ifndef LIDARSLAMMAPWIDGET_H
#define LIDARSLAMMAPWIDGET_H

#include <QWidget>
#include <QDebug>
#include <QColor>
#include <QImage>
#include "LidarSlamManager.h"
#include "LidarSlamCommon.h"

// #include <QtDataVisualization/Q3DScatter>
// #include <QtCharts/QScatterSeries>
// #include <QtCharts/QChartView>

#include <qtcustomplot/QCPColorGraph.h>

#include <QMutex>
#include <random>
#include <QOpenGLContext>

#include <qtcustomplot/qcustomplot.h>

namespace Ui
{
    class LidarSlamMapWidget;
}

class LidarSlamMapWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LidarSlamMapWidget(LidarSlamManager *manager,
                                QWidget *parent = nullptr);
    ~LidarSlamMapWidget();

    // Q_SIGNALS:
    QCustomPlot *getQCustomPlotWidget();

    QPixmap getPixmap();

public Q_SLOTS:
    void update();




private:
    Ui::LidarSlamMapWidget *ui;
    LidarSlamManager *m_Manager;
    LaserScan scan_data_;

    Odometry m_odom;
    Map m_map;

    Pose pose_;

    double radiation_level_;

    QMutex plot_mutex_;

    QTimer *timer_map_update_;

    QCPColorMap *colorMap;

    QVector<double> traj_x_, traj_y_;
    std::vector<QPen> colors_radiations_M;

    QVector<QColor> colors;

    double fRand(double fMin, double fMax);

    void plotWorldMapChart(Map &m);
    void plotWorldMap(const Pose &pose,const std::vector<QPoints> &map_xyz, std::pair<QVector<double>, QVector<double>> &traj);
};

#endif // LIDARSLAMMAPWIDGET_H
