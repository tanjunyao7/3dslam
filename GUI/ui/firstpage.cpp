#include "firstpage.h"
#include "ui_firstpage.h"

FirstPage::FirstPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FirstPage)
{
    ui->setupUi(this);
}

FirstPage::~FirstPage()
{
    delete ui;
}
