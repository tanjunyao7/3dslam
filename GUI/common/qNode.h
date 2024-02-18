#ifndef NODE_HPP_
#define NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <QtGui>

#include <nav_msgs/OccupancyGrid.h>

#include <LidarSlamCommon.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud2.h>


#include <pcl_conversions/pcl_conversions.h>



/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
    Q_OBJECT
public:
    QNode();
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    //    void run();
    virtual void run() = 0; // if we want to implement different task.

    void callStartStopNodes(bool start);

    /*********************
    ** Logging
    **********************/
    enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    QImage _image;
    QStringListModel *loggingModel() { return &logging_model; }
    void log(const LogLevel &level, const std::string &msg);

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void pose_callback(const nav_msgs::Odometry &msg);
    void radiation_callback(const std_msgs::Int32 &msg);

    void depth_callback(const sensor_msgs::ImageConstPtr &msg);
    void color_callback(const sensor_msgs::ImageConstPtr &msg);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &input);
    void path_callback(const nav_msgs::PathConstPtr& path);

    LaserScan scan_data;

    Map map;

    std::string package_path_;


    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cld_ptr;

    QPixmap px_rgb;
    QPixmap PixmapModel_rgb() { return px_rgb; }
    QPixmap px_depth;
    QPixmap PixmapModel_depth() { return px_depth; }

    QImage cvtCvMat2QImage(const cv::Mat &image);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

    void mapUpdated(Map &);
    void poseUpdated(Pose &);
    void radiationUpdated(int);

    void rgb_update(const QPixmap image);
    void depth_update(const QPixmap image);

    void cloud_update(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void path_update(const std::vector<std::pair<double,Eigen::Vector3d>>& path);

private:
    int init_argc;
    char **init_argv;

    std::unique_ptr<ros::NodeHandle> n;

    QStringListModel logging_model;


    std::string map_topic_;
    std::string pose_topic_;
    std::string radiation_topic_;
    std::string rgb_topic_;
    std::string depth_topic_;
    std::string cloud_topic_;
    std::string path_topic_;

    image_transport::Subscriber color_subscriber_, depth_subsciber_;
    ros::Subscriber laser_subscriber_, map_subscriber_, pose_subscriber_, radiation_subscriber_, cloud_subscriber_, path_subscriber_;

protected:
    virtual void ros_comms_init() = 0;
};

#endif /* NODE_HPP_ */
