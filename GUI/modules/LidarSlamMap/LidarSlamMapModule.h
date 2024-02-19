#ifndef LIDARSLAMMAPMODULE_H
#define LIDARSLAMMAPMODULE_H

#include <ros/ros.h>

#include <LidarSlamApplication.h>
#include <LidarSlamManager.h>
#include <LidarSlamModuleBase.h>
#include <LidarSlamMapWidget.h>


#include <QObject>
#include <QThread>

// #include <listener.h>

class LidarSlamMapWidget;

class LidarSlamMapModule : public QObject, public LidarSlamModuleBase
{
    Q_OBJECT
public:
    LidarSlamMapModule(LidarSlamManager *manager);
    ~LidarSlamMapModule();
    static LidarSlamModuleBase *createInstance(LidarSlamManager *manager = nullptr);

    void showModuleUI(LidarSlamMainView *view, bool visible);
    bool reqPreviousModule(void) override;
    bool reqNextModule(void) override;
    void setActive(bool) override;

    // public Q_SLOTS:

private:
    LidarSlamMapWidget *m_LidarSlamMapWidget = nullptr;
    LaserScan scan;
    Map map;
};

#endif // LIDARSLAMMAPMODULE_H
