#include "LidarSlamManager.h"
#include "pcl/io/pcd_io.h"

LidarSlamManager::LidarSlamManager(QObject *parent) : QObject(parent), m_activeModule(nullptr), m_dataMenuVisible(true), radiation_level_(0), load_data_(false), first_start_(true), cld_ptr(new pcl::PointCloud<pcl::PointXYZI>)
{
    ros_node_ = new Listener();
    // Connect to ROS
    if (!ros_node_->init())
    {
        ROS_INFO("NO MASTER FOUND");
    }
    else
    {
        ROS_INFO("ROS INIT");
        package_path_ = ros_node_->package_path_;
        QObject::connect(ros_node_, SIGNAL(mapUpdated(Map &)), this, SLOT(setMapData(Map &)), Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(poseUpdated(Pose &)), this, SLOT(setPoseData(Pose &)), Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(radiationUpdated(int)), this, SLOT(setRadiationData(int)), Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(rgb_update(const QPixmap)), this, SLOT(updatePixmapColor(const QPixmap)), Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(depth_update(const QPixmap)), this, SLOT(updatePixmapDepth(const QPixmap)), Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(cloud_update(pcl::PointCloud<pcl::PointXYZI>::Ptr)), this, SLOT(updateCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr)), Qt::DirectConnection);
        QObject::connect(ros_node_,SIGNAL(path_update(const std::vector<std::pair<double, Eigen::Vector3d>> &)),this,SLOT(updatePath(const std::vector<std::pair<double, Eigen::Vector3d>> &)),Qt::DirectConnection);
    }

}
LidarSlamManager::~LidarSlamManager()
{
    if (survey_file_->isOpen())
    {
        survey_file_->close();
        delete survey_file_;
    }
}

void LidarSlamManager::startUI(LidarSlamMainView *mainView)
{
    m_mainview = mainView;
    // setSurveyFile();
    setActiveModule(0, true); // home
                              //    setActiveModule(1); // User
                              //    setActiveModule(2); // Map
    // setActiveModule(3, false); // Data
}

bool LidarSlamManager::registerModule(LidarSlamModuleBase *module)
{
    module->setManager(this);
    m_modules.append(module);
    return true;
}

void LidarSlamManager::setActiveModule(int index, bool visible)
{
    if ((m_modules.count() <= index) || (index < 0))
        return;

    emit busyChanged(true);

    if (m_activeModule)
    {
        m_activeModule->setActive(false);
    }

    m_activeModule = m_modules.at(index);

    m_activeModule->showModuleUI(m_mainview, visible);
    m_activeModuleIndex = index;
    m_activeModule->setActive(true);

    QStringList heads;
    for (int i = 0; i <= m_activeModuleIndex; i++)
    {
        heads << m_modules.at(i)->moduleTitle();
    }
    setActiveModuleHeads(heads);
    emit busyChanged(false);
}

const QStringList &LidarSlamManager::activeModuleHeads() const
{
    return m_activeModuleHeads;
}

void LidarSlamManager::setActiveModuleHeads(const QStringList &newActiveModuleHeads)
{
    if (m_activeModuleHeads == newActiveModuleHeads)
        return;
    m_activeModuleHeads = newActiveModuleHeads;
    emit activeModuleHeadsChanged();
}

void LidarSlamManager::startSurvey(bool can_start)
{
    can_start_ = can_start;
    if (can_start_)
    {
        printf("start survey\n");
        setActiveModule(3, false);
        slam_data_.clear();
        ros_node_->callStartStopNodes(true);
    }
    else{
        printf("stopped survey\n");
        ros_node_->callStartStopNodes(false);
    }
    Q_EMIT start_survey(can_start_);
}

void LidarSlamManager::updatePath(const std::vector<std::pair<double,Eigen::Vector3d>>& path) {
    path_ = path;
    Q_EMIT path_update(path);
}

void LidarSlamManager::updateCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    cld_ptr = cloud;
    // Calculate average x, y, z from the cloud data
    Q_EMIT cloud_update(cloud);
}

void LidarSlamManager::updatePixmapColor(const QPixmap pix_image)
{
    color_image_ = pix_image;
    Q_EMIT rgb_update(color_image_);
}

void LidarSlamManager::updatePixmapDepth(const QPixmap pix_image) {
    depth_image_ = pix_image;
    Q_EMIT depth_update(depth_image_);
}

void LidarSlamManager::setLaserData(LaserScan &scan)
{
    if (scan.ranges.size() > 0)
    {
        scan_data_ = scan;
    }
}


double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void LidarSlamManager::setMapData(Map &map)
{
    if (map.data.size() > 0)
    {
        map_ = map;
        xyz_ = mapToWorld(map_);



        if (!pose_.position.empty())
        {
            traj_x_ << pose_.position.at(0);
            traj_y_ << pose_.position.at(1);
            traj_z_ << pose_.position.at(2);
            traj_2D = std::make_pair(traj_x_, traj_y_);

            QColor color;
            // auto radiation_level_ = fRand(0.0,4000.0);
            if ((radiation_level_ / 1000) == 0)
            {
                color = Qt::green;
            }
            else if ((radiation_level_ / 1000) < 1.95)
            {
                color = QColor(1, 100, 32);
            }
            else if (
                (radiation_level_ / 1000) >= 1.95 && (radiation_level_ / 1000) < 2.2)
            {
                color = Qt::yellow;
            }
            else if (
                (radiation_level_ / 1000) >= 2.2 && (radiation_level_ / 1000) <= 2.35)
            {
                color = QColor(255, 192, 203);
            }
            else
            {
                color = Qt::red;
            }
            colors_radiations_.push_back(color);
        }
    }
}

void LidarSlamManager::setRadiationData(int rad_data)
{
    std::cout<<"radiation: "<<rad_data<<std::endl;
    radiation_level_ = rad_data;
}

void LidarSlamManager::setPoseData(Pose &pose)
{
    // Update pose
    pose_ = pose;
}

std::vector<QPoints> LidarSlamManager::mapToWorld(Map &m)
{
    std::vector<QPoints> xyz;
    xyz.resize(2);

    for (int width = 0; width < m.width; ++width)
    {
        for (int height = 0; height < m.height; ++height)
        {
            if (m.data[height * m.width + width] > 0)
            {
                xyz.at(0).x.append((width * m.resolution + m.resolution / 2) + m.position_x);
                xyz.at(0).y.append((height * m.resolution + m.resolution / 2) + m.position_y);
                xyz.at(0).z.append(0.0);
            }
            else if (m.data[height * m.width + width] == 0)
            {
                xyz.at(1).x.append((width * m.resolution + m.resolution / 2) + m.position_x);
                xyz.at(1).y.append((height * m.resolution + m.resolution / 2) + m.position_y);
                xyz.at(1).z.append(0.0);
            }
        }
    }


    return xyz;
}

void LidarSlamManager::setSurveyFile(QString file_name)
{
    QString logFilePath = QString::fromStdString(package_path_) + "/log/";
    if (!QDir(logFilePath).exists())
        QDir().mkdir(logFilePath);

    logFilePath += file_name; // "data.csv";
    QString txt;
    survey_file_ = new QFile(logFilePath);
    survey_file_->open(QIODevice::WriteOnly | QIODevice::Text);
}

void LidarSlamManager::savePointCloud(const std::string& filename) {
    pcl::io::savePCDFileBinary(filename,*cld_ptr);
}
