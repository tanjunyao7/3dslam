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
    customPlot->xAxis->setScaleRatio(customPlot->yAxis);
    customPlot->clearGraphs();
    QCPGraph * graph;
    
    QVector<double> x_pix, y_pix;

    if ((!pose.position.empty()) && (!map_xyz.empty()))
    {
        x_pix << pose.position.at(0);
        y_pix << pose.position.at(1);

        graph = customPlot->addGraph();
        graph->setPen(QPen(Qt::black));
        graph->setLineStyle(QCPGraph::lsNone);
        graph->setScatterStyle(QCPScatterStyle::ssDisc);
        graph->setData(map_xyz.at(0).x, map_xyz.at(0).y);

        graph = customPlot->addGraph();
        graph->setPen(QPen(QColor(255, 0, 255)));
        graph->setLineStyle(QCPGraph::lsNone);
        graph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
        graph->setData(x_pix, y_pix);

        for(int i=0;i<m_Manager->colors_radiations_.size();i++){
            graph = customPlot->addGraph();
            graph->setPen(QPen(m_Manager->colors_radiations_[i]));
            graph->setLineStyle(QCPGraph::lsNone);
            graph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
            graph->addData(traj.first[i], traj.second[i]);
        }

        customPlot->replot(QCustomPlot::rpQueuedReplot); // refresh priority QCustomPlot::rpQueuedReplot)


    }
}

double LidarSlamMapWidget::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}


