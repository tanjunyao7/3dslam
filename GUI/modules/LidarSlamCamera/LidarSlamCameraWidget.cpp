#include "LidarSlamCameraWidget.h"
#include "ui_LidarSlamCameraWidget.h"
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

LidarSlamCameraWidget::LidarSlamCameraWidget(LidarSlamManager *manager, QWidget *parent) : QWidget(parent),
                                                                                           m_Manager(manager),
                                                                                           ui(new Ui::LidarSlamCameraWidget)
{
    ui->setupUi(this);
    timer_camera_update_ = new QTimer();
    connect(timer_camera_update_, SIGNAL(timeout()), this, SLOT(updateCloud()), Qt::DirectConnection);
    timer_camera_update_->start(update_rate_);

    connect(manager,SIGNAL(saveCloudScreen(const QString&)),this, SLOT(saveCloudScreenShot(const QString&)),Qt::DirectConnection);

    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->resetCamera();
}

LidarSlamCameraWidget::~LidarSlamCameraWidget()
{
    delete ui;
}

void LidarSlamCameraWidget::updateCloud()
{
    if (m_Manager->cld_ptr->points.size() > 0)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cut_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for(int i=0;i<m_Manager->cld_ptr->size();i++)
            if(m_Manager->cld_ptr->points[i].z<ui->max_height_spinbox->value())
                cut_cloud->push_back(m_Manager->cld_ptr->points[i]);

        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cut_cloud, "intensity");
        viewer_->addPointCloud(cut_cloud, intensity_distribution, "cloud");

        pcl::PointCloud<pcl::PointXYZ>::Ptr path_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto i : m_Manager->path_)
        {
            pcl::PointXYZ p;
            p.getVector3fMap() = i.second.cast<float>();
            path_cloud->push_back(p);
        }
        viewer_->addPointCloud(path_cloud, "path");
        float pointSize = ui->pointsize_spinbox->value();
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "path");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "path");
        ui->qvtkWidget->update();

    }
}

void LidarSlamCameraWidget::saveCloudScreenShot(const QString &dir) {

    // Get bounding box dimensions
    pcl::PointXYZI min_point, max_point;
    pcl::getMinMax3D(*m_Manager->cld_ptr, min_point, max_point);

    // Calculate optimal camera distance
    double max_dim = std::max({max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z});

    // Set camera position for the top view
    viewer_->setCameraPosition(0, 0, 1.5*max_dim, 0, 0, 0, 0, 1, 0);

    // Save top view screenshot
//    viewer_->setSize(640, 480);
    viewer_->saveScreenshot(dir.toStdString() + "top_view.png");

    // Set camera position for the third-person view
    viewer_->setCameraPosition(max_point.x, max_point.y, max_point.z, 0, 0, 0, -max_point.x, -max_point.y, max_point.z);

    // Save third-person view screenshot
//    viewer_->setSize(640, 480);
    viewer_->saveScreenshot(dir.toStdString() +"third_person_view.png");
}



