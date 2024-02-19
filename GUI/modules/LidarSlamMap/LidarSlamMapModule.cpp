#include "LidarSlamMapModule.h"

LidarSlamMapModule::LidarSlamMapModule(LidarSlamManager *manager) : QObject(NULL), LidarSlamModuleBase(manager, "MAP")
{

    // map_widget_ = new QThread();
}

LidarSlamMapModule::~LidarSlamMapModule()
{
    // map_widget_->quit();
    // map_widget_->wait();
}

LidarSlamModuleBase *LidarSlamMapModule::createInstance(LidarSlamManager *manager)
{
    return new LidarSlamMapModule(manager);
}

void LidarSlamMapModule::showModuleUI(LidarSlamMainView *view, bool visible)
{
    if (!m_LidarSlamMapWidget )
        m_LidarSlamMapWidget = new LidarSlamMapWidget(p_manager);

    view->setMainWidget(m_LidarSlamMapWidget);
}

bool LidarSlamMapModule::reqPreviousModule()
{
    return false;
}

bool LidarSlamMapModule::reqNextModule()
{
    return false;
}

void LidarSlamMapModule::setActive(bool active)
{
    if (active != m_active)
    {
        m_active = active;
    }
}
