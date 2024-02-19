#include "LidarSlamDataWidget.h"
#include "ui_LidarSlamDataWidget.h"

LidarSlamDataWidget::LidarSlamDataWidget(LidarSlamManager *manager, QWidget *parent) : QWidget(parent),
                                                                                       m_Manager(manager),
                                                                                       update_table_(true),
                                                                                       ui(new Ui::LidarSlamDataWidget)
{
    ui->setupUi(this);

    timer_map_update_ = new QTimer();

    connect(timer_map_update_, SIGNAL(timeout()), this, SLOT(update()), Qt::DirectConnection);
    timer_map_update_->start(update_rate_); // no ms

    // Set the edit triggers to disable editing
    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

LidarSlamDataWidget::~LidarSlamDataWidget()
{
    delete ui;
}

void LidarSlamDataWidget::update()
{
    if (m_Manager->load_data_ && update_table_)
    {
        ui->tableWidget->setRowCount(0);
        for (int i = 0; i < m_Manager->traj_x_.size(); i++)
        {
            int row = ui->tableWidget->rowCount();
            ui->tableWidget->insertRow(row);

            QTableWidgetItem *lidar_x = new QTableWidgetItem(QString::number(m_Manager->traj_x_.at(i)));
            int column_x = 0;
            ui->tableWidget->setItem(row, column_x, lidar_x);

            QTableWidgetItem *lidar_y = new QTableWidgetItem(QString::number(m_Manager->traj_y_.at(i)));
            int column_y = 1;
            ui->tableWidget->setItem(row, column_y, lidar_y);

            QTableWidgetItem *lidar_z = new QTableWidgetItem(QString::number(m_Manager->traj_z_.at(i)));
            int column_z = 2;
            ui->tableWidget->setItem(row, column_z, lidar_z);

            QTableWidgetItem *rad_level = new QTableWidgetItem(QString::number(m_Manager->radiations_.at(i)));
            int column_rad = 3;
            ui->tableWidget->setItem(row, column_rad, rad_level);
        }

        // timer_map_update_->stop(); // check this
        update_table_ = false;
    }
    else if (!m_Manager->load_data_)
    {
        if (m_Manager->started)
        {
            if (!update_table_)
            {
                ui->tableWidget->setRowCount(0);
                update_table_ = true;
            }
            //if (!timer_map_update_->isActive())
            //{
            //timer_map_update_->start(update_rate_);
            //}

            if (m_Manager->pose_.position.size() > 0)
            {
                int row = ui->tableWidget->rowCount();
                ui->tableWidget->insertRow(row);

                QTableWidgetItem *lidar_x = new QTableWidgetItem(QString::number(m_Manager->pose_.position.at(0)));
                int column_x = 0;
                ui->tableWidget->setItem(row, column_x, lidar_x);

                QTableWidgetItem *lidar_y = new QTableWidgetItem(QString::number(m_Manager->pose_.position.at(1)));
                int column_y = 1;
                ui->tableWidget->setItem(row, column_y, lidar_y);

                QTableWidgetItem *lidar_z = new QTableWidgetItem(QString::number(m_Manager->pose_.position.at(2)));
                int column_z = 2;
                ui->tableWidget->setItem(row, column_z, lidar_z);

                QTableWidgetItem *rad_level = new QTableWidgetItem(QString::number(m_Manager->radiation_level_ / 1000));
                int column_rad = 3;
                ui->tableWidget->setItem(row, column_rad, rad_level);

                QString curr_Data = QString::number(m_Manager->pose_.position.at(0)) + ";" +
                                    QString::number(m_Manager->pose_.position.at(1)) + ";" +
                                    QString::number(m_Manager->pose_.position.at(2)) + ";" + QString::number(m_Manager->radiation_level_ / 1000);

                m_Manager->slam_data_.push_back(curr_Data);
                //   writeCSV(QString::number(m_Manager->pose_.position.at(0)), QString::number(m_Manager->pose_.position.at(1)),
                //            QString::number(0.0), QString::number(m_Manager->radiation_level_ / 1000));
                ui->tableWidget->scrollToBottom();
            }
        }
    }
}

void LidarSlamDataWidget::writeUserToCSV()
{
    // if (m_Manager->survey_file_->open(QIODevice::WriteOnly | QIODevice::Text ))

    if (m_Manager->survey_file_->isOpen())
    {
        QTextStream ts(m_Manager->survey_file_);

        ts << "Survey Technician Name : ;" << m_Manager->user_.username << endl;
        ts << "Project / Client ID : ;" << m_Manager->user_.client_id << endl;
        ts << "Building ID : ;" << m_Manager->user_.building_id << endl;
        ts << "Survey Unit # : ;" << m_Manager->user_.survey_unit << endl;
        ts << "Instrument Info : ;" << m_Manager->user_.instrument << endl;
        ts << "Probe : ;" << m_Manager->user_.probe << endl;
        ts << "Radiation Type : ;" << m_Manager->user_.radiation_type << endl;
        ts << "Additional Comments and Information About Survey : ;" << m_Manager->user_.comment << endl;
        ts << endl;
        ts << "X ;"
           << "Y ;"
           << "Z ;"
           << "RADIATION ;" << endl;
    }
    else
    {
        std::cout << " CANT OPEN THE FILE" << std::endl;
    }
}

void LidarSlamDataWidget::writeCSV(QString x, QString y, QString z, QString rad)
{
    //  if (m_Manager->survey_file_->open(QIODevice::WriteOnly | QIODevice::Text))
    //  {
    QTextStream ts(m_Manager->survey_file_);

    ts << x << ";" << y << ";" << z << ";" << rad << endl;
    //  }
    //   else
    //   {
    //       std::cout << " CANT OPEN THE FILE" << std::endl;
    // }
}
