#include "LidarSlamManager.h"
#include "pcl/io/ply_io.h"
#include <QMessageBox>
#include <QFileDialog>
#include <cstdlib>

LidarSlamManager::LidarSlamManager(QObject *parent) : QObject(parent), m_activeModule(nullptr), m_dataMenuVisible(true),
                                                      radiation_level_(0), load_data_(false), first_start_(true),
                                                      cld_ptr(new pcl::PointCloud<pcl::PointXYZI>) {
    ros_node_ = new QNode();
    // Connect to ROS
    if (!ros_node_->init()) {
        ROS_INFO("NO MASTER FOUND");
    } else {
        ROS_INFO("ROS INIT");
        QObject::connect(ros_node_, SIGNAL(mapUpdated(Map & )), this, SLOT(setMapData(Map & )), Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(poseUpdated(Pose & )), this, SLOT(setPoseData(Pose & )),
                         Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(radiationUpdated(int)), this, SLOT(setRadiationData(int)),
                         Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(rgb_update(const QPixmap)), this, SLOT(updatePixmapColor(const QPixmap)),
                         Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(depth_update(const QPixmap)), this, SLOT(updatePixmapDepth(const QPixmap)),
                         Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(cloud_update(pcl::PointCloud<pcl::PointXYZI>::Ptr)), this,
                         SLOT(updateCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr)), Qt::DirectConnection);
        QObject::connect(ros_node_, SIGNAL(path_update(const std::vector<std::pair<double, Eigen::Vector3d>> &)), this,
                         SLOT(updatePath(const std::vector<std::pair<double, Eigen::Vector3d>> &)),
                         Qt::DirectConnection);
    }

}

LidarSlamManager::~LidarSlamManager() {
    if (survey_file_->isOpen()) {
        survey_file_->close();
        delete survey_file_;
    }
}

void LidarSlamManager::startUI(LidarSlamMainView *mainView) {
    m_mainview = mainView;
    // setSurveyFile();
    setActiveModule(0, true); // home
    //    setActiveModule(1); // User
    //    setActiveModule(2); // Map
    // setActiveModule(3, false); // Data
}

bool LidarSlamManager::registerModule(LidarSlamModuleBase *module) {
    module->setManager(this);
    m_modules.append(module);
    return true;
}

void LidarSlamManager::setActiveModule(int index, bool visible) {
    if ((m_modules.count() <= index) || (index < 0))
        return;


    if (m_activeModule) {
        m_activeModule->setActive(false);
    }

    m_activeModule = m_modules.at(index);

    m_activeModule->showModuleUI(m_mainview, visible);
    m_activeModuleIndex = index;
    m_activeModule->setActive(true);

    QStringList heads;
    for (int i = 0; i <= m_activeModuleIndex; i++) {
        heads << m_modules.at(i)->moduleTitle();
    }
    setActiveModuleHeads(heads);
}

const QStringList &LidarSlamManager::activeModuleHeads() const {
    return m_activeModuleHeads;
}

void LidarSlamManager::setActiveModuleHeads(const QStringList &newActiveModuleHeads) {
    if (m_activeModuleHeads == newActiveModuleHeads)
        return;
    m_activeModuleHeads = newActiveModuleHeads;
}

void LidarSlamManager::startSurvey(bool can_start) {
    started = can_start;
    if (started) {
        printf("start survey\n");
        clearData();
        load_data_ = false;

        QString log_dir = QString::fromStdString(ros_node_->package_path_) + "/log/";
        if (!QDir(log_dir).exists()) {
            QDir().mkdir(log_dir);
        }

        save_dir = log_dir + QString(user_.survey_unit) + "/";
        if (!QDir(save_dir).exists()) {
            QDir().mkdir(save_dir);
        }

        setSurveyFile();

        setActiveModule(3, false);
        ros_node_->callStartStopNodes(true);
    } else {
        printf("stopped survey\n");
        ros_node_->callStartStopNodes(false);

        saveResult();
    }
    Q_EMIT enable_buttons(started);
}

void LidarSlamManager::updatePath(const std::vector<std::pair<double, Eigen::Vector3d>> &path) {
    if (started)
        path_ = path;
//    Q_EMIT path_update(path);
}

void LidarSlamManager::updateCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    if (started)
        cld_ptr = cloud;
}

void LidarSlamManager::updatePixmapColor(const QPixmap pix_image) {
    if (started)
        color_image_ = pix_image;
}

void LidarSlamManager::updatePixmapDepth(const QPixmap pix_image) {
    if (started)
        depth_image_ = pix_image;
//    Q_EMIT depth_update(depth_image_);
}


double fRand(double fMin, double fMax) {
    double f = (double) rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

void LidarSlamManager::setMapData(Map &map) {
    if (started && map.data.size() > 0) {
        map_ = map;
        xyz_ = mapToWorld(map_);


        if (!pose_.position.empty()) {
            traj_x_ << pose_.position.at(0);
            traj_y_ << pose_.position.at(1);
            traj_z_ << pose_.position.at(2);
            traj_2D = std::make_pair(traj_x_, traj_y_);

            QColor color;
            // auto radiation_level_ = fRand(0.0,4000.0);
            if ((radiation_level_ / 1000) == 0) {
                color = Qt::green;
            } else if ((radiation_level_ / 1000) < 1.95) {
                color = QColor(1, 100, 32);
            } else if (
                    (radiation_level_ / 1000) >= 1.95 && (radiation_level_ / 1000) < 2.2) {
                color = Qt::yellow;
            } else if (
                    (radiation_level_ / 1000) >= 2.2 && (radiation_level_ / 1000) <= 2.35) {
                color = QColor(255, 192, 203);
            } else {
                color = Qt::red;
            }
            colors_radiations_.push_back(color);
        }
    }
}

void LidarSlamManager::setRadiationData(int rad_data) {
    if (started) {
        radiation_level_ = rad_data;
        radiations_.push_back(rad_data * 0.001f);
    }

}

void LidarSlamManager::setPoseData(Pose &pose) {
    if (started)
        pose_ = pose;
}


void LidarSlamManager::setSurveyFile() {
    QString file_name = "data_" + user_.client_id + "_" + user_.building_id + ".csv";
    logFilePath_ = save_dir + file_name; // "data.csv";
    std::cout << logFilePath_.toStdString() << std::endl;
    survey_file_ = new QFile(logFilePath_);
    survey_file_->open(QIODevice::WriteOnly | QIODevice::Text);
}


void LidarSlamManager::saveResult() {
    emit saveCloudScreen(save_dir);

    QString occupancy = save_dir + "occupancy_" + user_.client_id + "_" + user_.building_id + ".png";
    writeCSV(save_dir, occupancy);

    //contour and heat map
    std::string script_path = ros_node_->package_path_ + "/contour_map.py";
    std::string cmd = "python3 " + script_path + " " + logFilePath_.toStdString();
    std::cout << cmd << std::endl;
    std::system(cmd.c_str());


    occupancy_image_ = createImage(map_);
    // occupancy = image_path + "occupancy_" + user_.client_id + "_" + user_.building_id + ".png";
    occupancy_image_.save(occupancy);


    QString image = save_dir + "map_image_" + user_.client_id + "_" + user_.building_id + ".png";
    map_image_.save(image);

    pcl::io::savePLYFileBinary(save_dir.toStdString() + "cloud.ply", *cld_ptr);


    QMessageBox msgBox;
    msgBox.setText(" CSV and MAP Image saved in the following path :\n" + save_dir);
    msgBox.exec();
}

void LidarSlamManager::writeCSV(const QString &dir, const QString &occupancy) {
    if (survey_file_->isOpen()) {
        QTextStream ts(survey_file_);

        ts << "Survey Technician Name : ;" << user_.username << endl;
        ts << "Project / Client ID : ;" << user_.client_id << endl;
        ts << "Building ID : ;" << user_.building_id << endl;
        ts << "Survey Unit # : ;" << user_.survey_unit << endl;
        ts << "Instrument Info : ;" << user_.instrument << endl;
        ts << "Probe : ;" << user_.probe << endl;
        ts << "Radiation Type : ;" << user_.radiation_type << endl;
        ts << "Additional Comments and Information About Survey : ;" << user_.comment << endl;
        ts << endl;
        ts << "Occupancy map : ;" << occupancy << endl;
        ts << "Resolution : ;" << map_.resolution << endl;
        ts << "Position : ;" << map_.position_x << " " << map_.position_y << endl;
        ts << endl;
        ts << "X ;"
           << "Y ;"
           << "Z ;"
           << "RADIATION ;"
           << "Date & Time(PST) ;"
                << "Minimum Radiation ;"
                << "Maximum Radiation ;"
                << "Mean Radiation ;"
                << "Std.Dev. Radiation ;"<< endl;
        if (!slam_data_.empty()) {
            double min_rad = radiations_[0], max_rad = radiations_[0], sum_rad = 0.0, sum_rad2 = 0.0;
            for (auto r: radiations_) {
                if (min_rad > r)
                    min_rad = r;
                if (max_rad < r)
                    max_rad = r;
                sum_rad += r;
                sum_rad2 += r * r;
            }
            double avg_rad = sum_rad / radiations_.size();
            double stddev = std::sqrt(sum_rad2/radiations_.size()-avg_rad*avg_rad);

            ts<<slam_data_[0]<<" "<<min_rad<<"; "<<max_rad<<"; "<<avg_rad<<"; "<<stddev<<endl;

            for (auto &d: slam_data_) {
                ts << d << endl;
            }
        }

        survey_file_->close();
        delete survey_file_;
        survey_file_ = nullptr;
    } else {
        std::cout << "wtf?\n" << std::endl;
        exit(0);
    }
}

QImage LidarSlamManager::createImage(const Map &m) {
    int grid_width = m.width;
    int grid_height = m.height;
    QImage image(grid_width, grid_height, QImage::Format_RGB888);
    for (int y = 0; y < grid_height; ++y) {
        for (int x = 0; x < grid_width; ++x) {
            double occupancyValue = m.data[y * grid_width + x];
            bool isOccupied = (occupancyValue > 0.5);
            QColor color = isOccupied ? Qt::black : Qt::white;
            image.setPixelColor(x, y, color);
        }
    }
    return image;
}

std::vector<QPoints> LidarSlamManager::mapToWorld(const Map &m) {
    std::vector<QPoints> xyz;
    xyz.resize(2);

    for (int width = 0; width < m.width; ++width) {
        for (int height = 0; height < m.height; ++height) {
            if (m.data[height * m.width + width] > 0) {
                xyz.at(0).x.append((width * m.resolution + m.resolution / 2) + m.position_x);
                xyz.at(0).y.append((height * m.resolution + m.resolution / 2) + m.position_y);
                xyz.at(0).z.append(0.0);
            } else if (m.data[height * m.width + width] == 0) {
                xyz.at(1).x.append((width * m.resolution + m.resolution / 2) + m.position_x);
                xyz.at(1).y.append((height * m.resolution + m.resolution / 2) + m.position_y);
                xyz.at(1).z.append(0.0);
            }
        }
    }


    return xyz;
}


void LidarSlamManager::loadData() {
//    traj_x_.clear();
//    traj_y_.clear();
//    traj_z_.clear();
//    radiations_.clear();
//    pose_.position.clear();
//    pose_.orientation.clear();
//    map_.data.clear();
//    xyz_.clear();
//
//    readCSV(path);
//    QImage occupancyGrid(occupancy);
//    std::vector<double> data;
//    data.reserve(occupancyGrid.width() * occupancyGrid.height());
//
//    for (int y = 0; y < occupancyGrid.height(); ++y)
//    {
//        for (int x = 0; x < occupancyGrid.width(); ++x)
//        {
//            QRgb pixelValue = occupancyGrid.pixel(x, y);
//            int red = qRed(pixelValue);
//            int green = qGreen(pixelValue);
//            int blue = qBlue(pixelValue);
//            double occupancyValue = (red + green + blue) / (3.0 * 255.0);
//            bool isOccupied = (occupancyValue > 0.5);
//            double pixelData = isOccupied ? 0.0 : 1.0;
//            data.push_back(pixelData);
//        }
//    }
//
//    Map m;
//    m.width = occupancyGrid.width();
//    m.height = occupancyGrid.height();
//    m.resolution = resolution;
//    m.position_x = position_x;
//    m.position_y = position_y;
//    m.data = data;
//
//    xyz_ = mapToWorld(m);
//
//    Pose actual_pose;
//    actual_pose.position.push_back(0.0);
//    actual_pose.position.push_back(0.0);
//    actual_pose.position.push_back(0.0);
//    actual_pose.orientation.push_back(0.0);
//    actual_pose.orientation.push_back(0.0);
//    actual_pose.orientation.push_back(0.0);
//    actual_pose.orientation.push_back(0.0);
//    pose_ = actual_pose;
//
//    if (occupancyGrid.isNull())
//    {
//        load_data_ = false;
//    }
//    else
//        load_data_ = true;

}

void LidarSlamManager::readCSV(QString file_name) {
//    QFile file(file_name);
//    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
//    {
//        QTextStream in(&file);
//
//        int line_count = 0;
//        while (!in.atEnd())
//        {
//            QString line = in.readLine();
//            QStringList fields = line.split(';');
//
//            if (line_count == 9)
//            {
//                for (const QString &field : fields)
//                    if (field.contains("png", Qt::CaseInsensitive))
//                    {
//                        occupancy = field;
//                    }
//            }
//            if (line_count == 10) // resolution
//            {
//                resolution = fields.at(1).toDouble();
//            }
//            if (line_count == 11)
//            {
//                QStringList parts = fields.at(1).split(' ');
//                position_x = parts[0].toDouble();
//                position_y = parts[1].toDouble();
//            }
//            if (line_count > 13 && line_count % 4 == 0)
//            {
//                traj_x_ << fields.at(0).toDouble();
//                traj_y_ << fields.at(1).toDouble();
//                traj_z_ << fields.at(2).toDouble();
//                radiations_.push_back(fields.at(3).toDouble() / 1000);
//            }
//            // for (const QString &field : fields)
//            // {
//            //     // Process the field value
//            //     if (field.contains("png", Qt::CaseInsensitive))
//            //     {
//            //         occupancy = field;
//            //         // std::cout << field.toStdString() << std::endl;
//            //     }
//            //     // ...
//            // }
//            line_count++;
//        }
//        file.close();
//        traj_2D = std::make_pair(traj_x_, traj_y_);
//    }
}

void LidarSlamManager::clearData() {
    traj_x_.clear();
    traj_y_.clear();
    traj_z_.clear();
    colors_radiations_.clear();
    radiations_.clear();
    pose_.position.clear();
    pose_.orientation.clear();
    map_.data.clear();
    xyz_.clear();
    slam_data_.clear();
}
