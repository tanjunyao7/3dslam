#ifndef LIDARSLAMVIDEOMODULE_H
#define LIDARSLAMVIDEOMODULE_H

#include <LidarSlamApplication.h>
#include <LidarSlamManager.h>
#include <LidarSlamModuleBase.h>
#include <LidarSlamVideoWidget.h>
#include <QObject>

class LidarSlamVideoWidget;

class LidarSlamVideoModule : public QObject, public LidarSlamModuleBase
{
    Q_OBJECT
public:
    LidarSlamVideoModule(LidarSlamManager *manager);
    static LidarSlamModuleBase *createInstance(LidarSlamManager *manager = nullptr);

    void showModuleUI(LidarSlamMainView *view, bool visible);
    bool reqPreviousModule(void) override;
    bool reqNextModule(void) override;
    void setActive(bool) override;

private:
    LidarSlamVideoWidget *m_LidarSlamVideoWidget = nullptr;
};

#endif // LIDARSLAMVIDEOMODULE_H
