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
    string img1Path;
    string img2Path;
    string outputImgPath;

    img1Path = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/Im7a.jpg";
    img2Path = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/Im7b.jpg";
    outputImgPath = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/Im7a-b.jpg";

    QImage inputImage1 = QImage(img1Path.c_str());
    QImage inputImage2 = QImage(img2Path.c_str());

    cout << "*************** INPUT ***************" << endl;
    cout << img1Path << endl;
    cout << img2Path << endl;
    cout << "*************** INPUT ***************" << endl;
    cout << endl;

    QVector< QVector<Vector3f> > lists = Utils::sift2(img1Path.c_str(),
                                                     img2Path.c_str());


}
