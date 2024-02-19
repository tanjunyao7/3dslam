#include "mainwindow.h"

MainWindow::MainWindow(LidarSlamManager *manager, QWidget *parent)
    : QMainWindow(parent), m_Manager(manager), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    initializeWindow();

    m_Manager->startUI(ui->mainFrame);

    ui->pb_Map->setEnabled(false);
    ui->pb_Data->setEnabled(false);
    ui->pb_Save->setEnabled(false);
    ui->pb_3D->setEnabled(false);
    ui->pb_Video->setEnabled(false);
    connect(m_Manager, SIGNAL(enable_buttons(bool)), this, SLOT(enable_buttons(bool)), Qt::DirectConnection);
    connect(this, SIGNAL(load_data(bool)), this, SLOT(enable_buttons(bool)), Qt::DirectConnection);


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initializeWindow()
{
    m_firstPage = new FirstPage();
    ui->mainFrame->setMainWidget(m_firstPage);
}

void MainWindow::on_pb_Home_clicked()
{
    m_Manager->setActiveModule(0, true);
}

void MainWindow::on_pb_User_clicked()
{
    m_Manager->setActiveModule(1, true);
}

void MainWindow::on_pb_Map_clicked()
{
    m_Manager->setActiveModule(2, true);
}

void MainWindow::on_pb_Data_clicked()
{
    m_Manager->setActiveModule(3, true);
}
void MainWindow::on_pb_3D_clicked()
{
    m_Manager->setActiveModule(4, true);
}

void MainWindow::on_pb_Video_clicked(){
    m_Manager->setActiveModule(5,true);
}

void MainWindow::on_pb_Save_clicked()
{
    m_Manager->saveResult();
}

void MainWindow::on_pb_Load_clicked()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Open the survey file"), "../", tr("csv File (*.csv)"));

    m_Manager->loadData();
    Q_EMIT load_data(m_Manager->load_data_);
}

void MainWindow::enable_buttons(bool enable)
{
    if (enable && m_Manager->first_start_)
    {
        ui->pb_Map->setEnabled(enable);
        ui->pb_Data->setEnabled(enable);
        ui->pb_Save->setEnabled(enable);
        ui->pb_3D->setEnabled(enable);
        ui->pb_Video->setEnabled(enable);
        ui->pb_Load->setDisabled(enable);
        m_Manager->first_start_ = false;
    }
    else
    {
        ui->pb_Load->setDisabled(enable);
    }
}



// > 0  wall ; == 0 free

