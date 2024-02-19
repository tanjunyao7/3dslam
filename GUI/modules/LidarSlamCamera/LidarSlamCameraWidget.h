#ifndef LIDARSLAMCAMERAWIDGET_H
#define LIDARSLAMCAMERAWIDGET_H
#include <pcl/search/impl/search.hpp>
#include <QWidget>
#include <QFileDialog>
#include <LidarSlamManager.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <vtkRenderWindow.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkDelaunay3D.h>
#include <vtkSmartPointer.h>
#include <vtkGeometryFilter.h>
#include <vtkCell.h>
#include <pcl/io/vtk_lib_io.h>


namespace Ui
{
    class LidarSlamCameraWidget;
}

class LidarSlamCameraWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LidarSlamCameraWidget(LidarSlamManager *manager, QWidget *parent = nullptr);
    ~LidarSlamCameraWidget();

public Q_SLOTS:
    void updateCloud();
    void saveCloudScreenShot(const QString& dir);



private:
    Ui::LidarSlamCameraWidget *ui;
    LidarSlamManager *m_Manager;

    QTimer *timer_camera_update_;
    int update_rate_ = 30;

protected:

    /** @brief The PCL visualizer object */
    pcl::visualization::PCLVisualizer::Ptr viewer_;

};

#endif // LIDARSLAMCAMERAWIDGET_H
