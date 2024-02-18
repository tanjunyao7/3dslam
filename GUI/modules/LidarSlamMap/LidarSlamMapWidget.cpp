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

    ui->tb_2dview->setChecked(true);
    ui->customPlot->setVisible(true);

    ui->label_lidar->setVisible(false);
    ui->label_lidar->setStyleSheet("QLabel{background-color:rgb(255,0,0); border-radius: 10px; min-height: 20px; min-width: 20px;max-width: 40px;}");

    // give the axes some labels:
    ui->customPlot->xAxis->setLabel("x");
    ui->customPlot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    ui->customPlot->xAxis->setRange(-5, 5);
    ui->customPlot->yAxis->setRange(-5, 5);
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->widget->setVisible(false);

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

QWidget *LidarSlamMapWidget::getUIWidget()
{
    return ui->widget;
}

QCustomPlot *LidarSlamMapWidget::getQCustomPlotWidget()
{
    return ui->customPlot;
}

void LidarSlamMapWidget::plotWorldMap(const Pose &pose,const std::vector<QPoints> &map_xyz, std::pair<QVector<double>, QVector<double>> &traj)
{
    ui->customPlot->clearGraphs();
    ui->customPlot->addGraph();
    QVector<double> x_pix, y_pix;

    if ((!pose.position.empty()) && (!map_xyz.empty()))
    {
        x_pix << pose.position.at(0);
        y_pix << pose.position.at(1);


        ui->customPlot->graph(0)->setPen(QPen(Qt::black));
        ui->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
        ui->customPlot->graph(0)->setData(map_xyz.at(0).x, map_xyz.at(0).y);

        ui->customPlot->addGraph();
        ui->customPlot->graph(1)->setPen(QPen(QColor(255, 0, 255)));
        ui->customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));
        ui->customPlot->graph(1)->setData(x_pix, y_pix);

        QCPColorGraph *graph = new QCPColorGraph(ui->customPlot->xAxis, ui->customPlot->yAxis);

        graph->setData(traj.first, traj.second, m_Manager->colors_radiations_);

        ui->customPlot->graph(2)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot->graph(2)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 10));

        ui->customPlot->replot(QCustomPlot::rpQueuedReplot); // refresh priority QCustomPlot::rpQueuedReplot)

    }
}


Points LidarSlamMapWidget::projectLaser(LaserScan &scan)
{
    Points xyz;
    double angle = scan.angle_min;
    // Spherical->Cartesian projection
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        xyz.x.push_back(cos(angle) * scan.ranges.at(i));
        xyz.y.push_back(sin(angle) * scan.ranges.at(i));
        angle += scan.angle_increment;
    }
    return xyz;
}

void LidarSlamMapWidget::plotLaserData(LaserScan &scan)
{
    if (scan.ranges.size() > 0)
    {
        ui->customPlot->addGraph();
        QVector<double> qVec_x, qVec_y;
        double angle = scan.angle_min;
        for (auto &pt : scan.ranges)
        {
            qVec_x.push_back(std::cos(angle) * pt);
            qVec_y.push_back(std::sin(angle) * pt);
            angle += scan.angle_increment;
        }

        ui->customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
        ui->customPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssStar);
        ui->customPlot->graph(0)->setData(qVec_x, qVec_y);

        // Position where you want the rectangle (centered at 0,0 in this example)
        double x_center = 0.0;
        double y_center = 0.0;
        double rect_half_width = 0.075;  // Adjust this for the rectangle's size
        double rect_half_height = 0.075; // Adjust this for the rectangle's size
        
        QCPItemRect *rectItem = new QCPItemRect(ui->customPlot);
        rectItem->topLeft->setCoords(x_center - rect_half_width, y_center + rect_half_height);
        rectItem->bottomRight->setCoords(x_center + rect_half_width, y_center - rect_half_height);
        rectItem->setBrush(QBrush(Qt::blue)); // Set rectangle color to red
        rectItem->setPen(QPen(Qt::NoPen)); // No border

        ui->customPlot->replot(QCustomPlot::rpQueuedRefresh);
    }
}



void LidarSlamMapWidget::plot3DView(Map &m, std::vector<QPoints> &map_xyz)
{
}

// CALLBACKS
void LidarSlamMapWidget::on_tb_surface_toggled(bool checked)
{
    if (checked)
    {
        std::cout << "SURFACE" << std::endl;
        ui->customPlot->setVisible(true);
    }
    else
    {
        ui->customPlot->setVisible(false);
    }
}

void LidarSlamMapWidget::on_tb_laser_toggled(bool checked)
{
    if (checked)
    {
        std::cout << " LASER VIEW" << std::endl;
        ui->widget->setVisible(false);
        ui->customPlot->setVisible(true);
        ui->customPlot->clearGraphs();
    }
    else
    {
        ui->customPlot->setVisible(false);
    }
}

void LidarSlamMapWidget::on_tb_2dview_toggled(bool checked)
{
    if (checked)
    {
        std::cout << " TAB VIEW" << std::endl;
        ui->widget->setVisible(false);
        ui->customPlot->setVisible(true);
        ui->customPlot->clearGraphs();
    }
    else
    {
        ui->customPlot->setVisible(false);
    }
}

void LidarSlamMapWidget::on_tb_3dview_toggled(bool checked)
{
    if (checked)
    {
        std::cout << " 3D VIEW" << std::endl;
        ui->widget->setVisible(true);
        ui->customPlot->setVisible(false);
        ui->customPlot->clearGraphs();
    }
    else
    {
        ui->customPlot->setVisible(true);
        ui->widget->setVisible(false);
    }
}

double LidarSlamMapWidget::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
