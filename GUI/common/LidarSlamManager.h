#ifndef LIDARSLAM_H
#define LIDARSLAM_H

#include <QObject>
#include <QSharedPointer>
#include <QDir>
#include <iostream>
#include <LidarSlamMainView.h>
#include <LidarSlamModuleBase.h>
#include <LidarSlamCommon.h>

#include <qNode.h>


typedef QList<LidarSlamModuleBase *> ModulesList;
class LidarSlamManager : public QObject
{
    Q_OBJECT

public:
    explicit LidarSlamManager(QObject *parent = nullptr);
    ~LidarSlamManager();
    bool registerModule(LidarSlamModuleBase *module);
    const ModulesList &modules();
    void startUI(LidarSlamMainView *mainView);
    void setActiveModule(int index, bool visible);

    const QStringList &activeModuleHeads() const;
    void setActiveModuleHeads(const QStringList &newActiveModuleHead);
    void setSurveyFile();
    void saveResult();
    void loadData();

    QString save_dir;
    QFile *survey_file_;
    Map map_;
    Pose pose_;
    User user_;
    bool started;
    bool first_start_;
    QPixmap map_image_;
    QImage occupancy_image_;

    QPixmap color_image_;
    QPixmap depth_image_;

    std::vector<QString> slam_data_;

    double radiation_level_;
    std::vector<double> radiations_;
    std::vector<QPoints> xyz_;
    bool load_data_;

    QVector<double> traj_x_, traj_y_, traj_z_;

    std::pair<QVector<double>, QVector<double>> traj_2D;
    
    QVector<QColor> colors_radiations_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cld_ptr;
    std::vector<std::pair<double,Eigen::Vector3d>> path_;

    std::vector<QPoints> mapToWorld(const Map &m);

public Q_SLOTS:
    void startSurvey(bool can_start);
    void setMapData(Map &map);
    void setPoseData(Pose &pose);
    void setRadiationData(int rad_data);
    void updatePixmapColor(const QPixmap);
    void updatePixmapDepth(const QPixmap);
    void updateCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void updatePath(const std::vector<std::pair<double,Eigen::Vector3d>>& path);

signals:
    void enable_buttons(bool);
    void saveCloudScreen(const QString& dir);

private:
    void clearData();
    void writeCSV(const QString& dir,const QString& occupancy);
    QImage createImage(const Map &m);

    void readCSV(QString file_name);

    LidarSlamMainView *m_mainview;
    bool m_dataMenuVisible;
    ModulesList m_modules;

    int m_activeModuleIndex;
    LidarSlamModuleBase *m_activeModule;
    QStringList m_activeModuleHeads;

    QNode *ros_node_;
    QString logFilePath_;
};

#endif // LIDARSLAM_H
