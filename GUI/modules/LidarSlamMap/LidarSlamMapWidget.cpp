#include "LidarSlamMapWidget.h"
#include "ui_LidarSlamMapWidget.h"

LidarSlamMapWidget::LidarSlamMapWidget(LidarSlamManager *manager, QWidget *parent) : QWidget(parent),
                                                                                     ui(new Ui::LidarSlamMapWidget),
                                                                                     m_Manager(manager),
                                                                                     radiation_level_(0)
{
    ui->setupUi(this);

    timer_map_update_ = new QTimer();

    connect(timer_map_update_, SIGNAL(timeout()), this, SLOT(update()), Qt::DirectConnection);
    timer_map_update_->start(150); // no ms

    ui->customPlot->setOpenGl(false);

    ui->customPlot->setVisible(true);

    // give the axes some labels:
    ui->customPlot->xAxis->setLabel("x");
    ui->customPlot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    ui->customPlot->xAxis->setRange(-5, 5);
    ui->customPlot->yAxis->setRange(-5, 5);
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

}

LidarSlamMapWidget::~LidarSlamMapWidget()
{
    delete ui;
    delete timer_map_update_;
}

QPixmap LidarSlamMapWidget::getPixmap()
{
    return ui->customPlot->toPixmap();
}

void LidarSlamMapWidget::update()
{
    radiation_level_ = m_Manager->radiation_level_ / 1000;

    plotWorldMap(m_Manager->pose_, m_Manager->xyz_, m_Manager->traj_2D);
    m_Manager->map_image_ = ui->customPlot->toPixmap();

}


QCustomPlot *LidarSlamMapWidget::getQCustomPlotWidget()
{
    return ui->customPlot;
}

void LidarSlamMapWidget::plotWorldMap(const Pose &pose,const std::vector<QPoints> &map_xyz, std::pair<QVector<double>, QVector<double>> &traj)
{
    QCustomPlot* customPlot = ui->customPlot;
    customPlot->clearGraphs();
    customPlot->addGraph();
    customPlot->addGraph();
    customPlot->addGraph();

    QVector<double> x_pix, y_pix;

    if ((!pose.position.empty()) && (!map_xyz.empty()))
    {
        x_pix << pose.position.at(0);
        y_pix << pose.position.at(1);
        std::cout<<"map size: "<<map_xyz[0].x.size()<<" "<<map_xyz[0].y.size()<<std::endl;

        customPlot->graph(0)->setPen(QPen(Qt::black));
        customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
        customPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
        customPlot->graph(0)->setData(map_xyz.at(0).x, map_xyz.at(0).y);

        customPlot->graph(1)->setPen(QPen(QColor(255, 0, 255)));
        customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
        customPlot->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
        customPlot->graph(1)->setData(x_pix, y_pix);

        QCPColorGraph *graph = new QCPColorGraph(customPlot->xAxis, customPlot->yAxis);
        graph->setData(traj.first, traj.second, m_Manager->colors_radiations_);
        graph->setLineStyle(QCPGraph::lsNone);
        graph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));

        customPlot->replot(QCustomPlot::rpQueuedReplot); // refresh priority QCustomPlot::rpQueuedReplot)

    }
}

double LidarSlamMapWidget::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}


