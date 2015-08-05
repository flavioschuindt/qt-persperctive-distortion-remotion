#include "auxiliarwindow.h"
#include "ui_auxiliarwindow.h"
#include <QMessageBox>

AuxiliarWindow::AuxiliarWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::AuxiliarWindow)
{
    ui->setupUi(this);
    ui->pictureContainer2->setBackgroundRole(QPalette::Base);
    ui->pictureContainer2->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->pictureContainer2->setScaledContents(true);
}

AuxiliarWindow::~AuxiliarWindow()
{
    delete ui;
}

void AuxiliarWindow::on_actionOpen_triggered()
{
    QImage imageObject;
    imageObject.load("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/2.png");
    ui->pictureContainer2->setPixmap(QPixmap::fromImage(imageObject));
    QSize size = ui->pictureContainer2->pixmap()->size();
    ui->pictureContainer2->resize(size.width(), size.height());
    this->resize(size.width(), size.height());

}
