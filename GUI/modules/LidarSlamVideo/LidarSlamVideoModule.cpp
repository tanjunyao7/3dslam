#include "LidarSlamVideoModule.h"

LidarSlamVideoModule::LidarSlamVideoModule(LidarSlamManager *manager) : QObject(NULL), LidarSlamModuleBase(manager, "CAMERA")
{
}

LidarSlamModuleBase *LidarSlamVideoModule::createInstance(LidarSlamManager *manager)
{
    return new LidarSlamVideoModule(manager);
}

void LidarSlamVideoModule::showModuleUI(LidarSlamMainView *view, bool visible)
{
    //    p_manager->setCanForward(true);

    if (!m_LidarSlamVideoWidget)
    {
        m_LidarSlamVideoWidget = new LidarSlamVideoWidget(p_manager);
    }
    view->setMainWidget(m_LidarSlamVideoWidget);

}

bool LidarSlamVideoModule::reqPreviousModule()
{
    return false;
}

bool LidarSlamVideoModule::reqNextModule()
{
    return false;
}

void LidarSlamVideoModule::setActive(bool active)
{
    if (active != m_active)
    {
        m_active = active;
    }
}
