/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/network.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include "qdebug.h"
#include "qNode.h"

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode()
{
}

QNode::~QNode()
{
    if (ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init()
{
    // ros::init(init_argc, init_argv, "lidar_slam_gui");
    if (!ros::master::check())
    {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    n.reset(new ros::NodeHandle);
    image_transport::ImageTransport it(*n);
    n->param("map_topic", map_topic_, std::string("/occupancy_grid"));
    n->param("pose_topic", pose_topic_, std::string("/aft_pgo_odom"));
    n->param("radiation_topic", radiation_topic_, std::string("/radiation"));
    n->param("rgb_topic", rgb_topic_, std::string("/rgb"));
    n->param("depth_topic", depth_topic_, std::string("/depth"));
    n->param("cloud_topic", cloud_topic_, std::string("/aft_pgo_map"));
    n->param("path_topic", path_topic_, std::string("/aft_pgo_path"));

    // n->param("cloud_topic", cloud_topic_, std::string("/camera/depth_registered/points"));
    

    map_subscriber_ = n->subscribe(map_topic_, 1000, &QNode::map_callback, this);
    pose_subscriber_ = n->subscribe(pose_topic_, 1000, &QNode::pose_callback, this);
    radiation_subscriber_ = n->subscribe(radiation_topic_, 1000, &QNode::radiation_callback, this);

    color_subscriber_ = it.subscribe(rgb_topic_, 1000, &QNode::color_callback, this);
    depth_subsciber_ = it.subscribe(depth_topic_, 1000, &QNode::depth_callback, this);
    cloud_subscriber_ = n->subscribe(cloud_topic_, 1, &QNode::cloud_callback, this);
    path_subscriber_ = n->subscribe(path_topic_,1,&QNode::path_callback,this);
    ros_comms_init();

    start();
    package_path_ = ros::package::getPath("LidarSLAM");
    return true;
}

void QNode::callStartStopNodes(bool start){
    ros::ServiceClient lio_client = n->serviceClient<std_srvs::SetBool>("start_stop_lio");
    ros::ServiceClient pgo_client = n->serviceClient<std_srvs::SetBool>("start_stop_pgo");
    std_srvs::SetBoolRequest request;
    request.data=start;
    std_srvs::SetBoolResponse response;

    lio_client.call(request,response);
    pgo_client.call(request,response);
}


void QNode::path_callback(const nav_msgs::PathConstPtr& path){
    std::vector<std::pair<double,Eigen::Vector3d>> p;
    for(auto pose:path->poses){
        double t = pose.header.stamp.toSec();
        Eigen::Vector3d position(&pose.pose.position.x);
        p.push_back({t,position});
    }
    Q_EMIT path_update(p);
}


void QNode::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PCLPointCloud2 pcl_pc2;             //struttura pc2 di pcl
    pcl_conversions::toPCL(*input, pcl_pc2); //conversione a pcl della pc2

    pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

   Q_EMIT cloud_update(cloud);
}

QImage QNode::cvtCvMat2QImage(const cv::Mat &image)
{
    QImage qtemp;
    if (!image.empty() && image.depth() == CV_8U)
    {
        const unsigned char *data = image.data;
        qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
        for (int y = 0; y < image.rows; ++y, data += image.cols * image.elemSize())
        {
            for (int x = 0; x < image.cols; ++x)
            {
                QRgb *p = ((QRgb *)qtemp.scanLine(y)) + x;
                *p = qRgb(data[x * image.channels() + 2], data[x * image.channels() + 1], data[x * image.channels()]);
            }
        }
    }
    else if (!image.empty() && image.depth() != CV_8U)
    {
        printf("Wrong image format, must be 8_bits\n");
    }
    return qtemp;
}

void QNode::color_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_INFO("Entered the CALLBACK() function");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        px_rgb = QPixmap::fromImage(cvtCvMat2QImage(cv_ptr->image));
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //-------------------------------------------------------
    Q_EMIT rgb_update(px_rgb);
}

void QNode::depth_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_INFO("Entered the CALLBACK() function");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr->image.convertTo(cv_ptr->image, CV_32F, 0.001);
        cv_ptr->image *= 255.0/10.0;
        cv_ptr->image.convertTo(cv_ptr->image, CV_8UC3);
        px_depth = QPixmap::fromImage(cvtCvMat2QImage(cv_ptr->image));
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //-------------------------------------------------------
    Q_EMIT depth_update(px_depth);
}


void QNode::pose_callback(const nav_msgs::Odometry &msg)
{
    // ROS_INFO_STREAM("POSE");
    Pose actual_pose;
    actual_pose.position.push_back(msg.pose.pose.position.x);
    actual_pose.position.push_back(msg.pose.pose.position.y);
    actual_pose.position.push_back(msg.pose.pose.position.z);
    actual_pose.orientation.push_back(msg.pose.pose.orientation.x);
    actual_pose.orientation.push_back(msg.pose.pose.orientation.y);
    actual_pose.orientation.push_back(msg.pose.pose.orientation.z);
    actual_pose.orientation.push_back(msg.pose.pose.orientation.w);

    Q_EMIT poseUpdated(actual_pose);
}


void QNode::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    Map _map;
    _map.width = msg->info.width;
    _map.height = msg->info.height;
    _map.resolution = msg->info.resolution;
    _map.position_x = msg->info.origin.position.x;
    _map.position_y = msg->info.origin.position.y;
    for (auto &m : msg->data)
    {
        _map.data.emplace_back(m);
    }

    Q_EMIT mapUpdated(_map);
}

void QNode::radiation_callback(const std_msgs::Int32 &msg)
{
    //std::cout << " msg data " << msg.data << std::endl;
    Q_EMIT radiationUpdated(msg.data);
}


/*****************************************************************************
** log
*****************************************************************************/
void QNode::log(const LogLevel &level, const std::string &msg)
{
    logging_model.insertRows(logging_model.rowCount(), 1);
    std::stringstream logging_model_msg;
    switch (level)
    {
    case (Debug):
    {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Info):
    {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Warn):
    {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Error):
    {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case (Fatal):
    {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount() - 1), new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

