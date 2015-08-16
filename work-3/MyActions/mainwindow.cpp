#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "utils.h"
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
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionOpen_triggered()
{
    QImage imageObject;
    imageObject.load("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite2.jpg");
    ui->pictureContainer->setPixmap(QPixmap::fromImage(imageObject));
    QSize size = ui->pictureContainer->pixmap()->size();
    ui->pictureContainer->resize(size.width(), size.height());
    this->resize(size.width(), size.height());
}

void MainWindow::on_actionSave_triggered()
{
    vector< pair<Dot*, Dot*> > pairs;
    Matrix3d H;
    if (METHOD == 0)
    {
        QList<Dot *> points = ui->pictureContainer->selectedPoints;
        for( int i=0; i < points.count(); i+=2 )
        {
            pair<Dot *, Dot *> pair(points.at(i), points.at(i+1));
            pairs.push_back(pair);
        }
    }
    else if (METHOD == 1)
    {
        /*vector< pair<Dot*, Dot*> > teste;
        teste = Utils::surf("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite2.jpg",
                    "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite1.jpg");
        pairs.push_back(teste.at(10));
        pairs.push_back(teste.at(66));
        pairs.push_back(teste.at(43));
        pairs.push_back(teste.at(70));*/
        pairs = Utils::surf("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite2.jpg",
                            "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite1.jpg");
        H = Utils::ransac(pairs, 4, 200);
    }

    QImage inputImage1 = QImage("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite2.jpg");
    QImage inputImage2 = QImage("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite1.jpg");
    //H = Utils::dltNormalized(pairs);

    vector< std::pair<QImage, Matrix3d> > inputPairs;
    cout << H << endl;
    inputPairs.push_back(std::pair<QImage, Matrix3d>(inputImage1, Matrix<double, 3, 3>::Identity()));
    inputPairs.push_back(std::pair<QImage, Matrix3d>(inputImage2, H));

    QImage outputImage = Utils::panoramic(inputPairs);
    Utils::saveImage(outputImage, "/home/fschuindt/teste.jpg");

}
