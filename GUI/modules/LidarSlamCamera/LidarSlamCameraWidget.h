#ifndef LIDARSLAMCAMERAWIDGET_H
#define LIDARSLAMCAMERAWIDGET_H
#include <pcl/search/impl/search.hpp> 
// #include <pcl/surface/impl/poisson.hpp>
// #include <pcl/visualization/impl/pcl_visualizer.hpp>
// #include <pcl/kdtree/impl/kdtree_flann.hpp>
// #include <pcl/visualization/impl/pcl_visualizer.hpp>
// #include <pcl/visualization/impl/pcl_visualizer.hpp>
// #include <pcl/visualization/impl/pcl_visualizer.hpp>
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
// #include <pcl/surface/poisson.h>
// #include <pcl/point_cloud.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/segmentation/extract_clusters.h>

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

Q_SIGNALS:
    void start_survey(bool can_start);
    void stop_survey(bool can_start);

public Q_SLOTS:
    void updateCloud();



private:
    Ui::LidarSlamCameraWidget *ui;
    LidarSlamManager *m_Manager;

    QTimer *timer_camera_update_;
    int update_rate_ = 30;
    // void setCameraData();

    bool camera_view;

protected:
    /** @brief Rerender the view */
    void refreshView();

    /** @brief The PCL visualizer object */
    pcl::visualization::PCLVisualizer::Ptr viewer_;

};

#endif // LIDARSLAMCAMERAWIDGET_H
