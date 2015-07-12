#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QLabel>
#include <Qt>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->pictureContainer->setBackgroundRole(QPalette::Base);
    ui->pictureContainer->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    ui->pictureContainer->setScaledContents(true);
    /*QFrame* frame = new QFrame(this);
    QRect rect(0, 0, 900, 600);
    frame->setFrameShape(QFrame::Box);
    frame->setGeometry(rect);*/
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionOpen_triggered()
{

    //QMessageBox::information(this, "title", "Hello");

    QImage imageObject;
    imageObject.load("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-2/MyActions/piso-afim-retas-paralelas.jpg");
    ui->pictureContainer->setPixmap(QPixmap::fromImage(imageObject));
    QSize size = ui->pictureContainer->pixmap()->size();
    ui->pictureContainer->resize(size.width(), size.height());
    this->resize(size.width(), size.height());
}
