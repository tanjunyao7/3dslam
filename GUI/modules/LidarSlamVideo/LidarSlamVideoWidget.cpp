#include "LidarSlamVideoWidget.h"
#include "ui_LidarSlamVideoWidget.h"

LidarSlamVideoWidget::LidarSlamVideoWidget(LidarSlamManager *manager, QWidget *parent) : 
    QWidget(parent),
    m_Manager(manager),
    ui(new Ui::LidarSlamVideoWidget)
{
    ui->setupUi(this);

    timer_camera_update_ = new QTimer();
    connect(timer_camera_update_, SIGNAL(timeout()), this, SLOT(updateVideoView()), Qt::DirectConnection);
    timer_camera_update_->start(update_rate_);

}

LidarSlamVideoWidget::~LidarSlamVideoWidget()
{
    delete ui;
}

void LidarSlamVideoWidget::updateVideoView()
{
    ui->lbl_Camera->setAlignment(Qt::AlignCenter);
    ui->lbl_Camera->setPixmap(m_Manager->color_image_);
    ui->lbl_Camera->repaint();

    ui->lbl_Depth->setAlignment(Qt::AlignCenter);
    ui->lbl_Depth->setPixmap(m_Manager->depth_image_);
    ui->lbl_Depth->repaint();
}


