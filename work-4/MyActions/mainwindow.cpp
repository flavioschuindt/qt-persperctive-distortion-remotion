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
    string img1Path;
    string img2Path;
    string outputImgPath;

    img1Path = "/home/vagrant/dev/qt-persperctive-distortion-remotion/work-4/MyActions/thai-lion/DSC_0176.JPG";
    img2Path = "/home/vagrant/dev/qt-persperctive-distortion-remotion/work-4/MyActions/thai-lion/DSC_0179.JPG";

    cout << "*************** INPUT ***************" << endl;
    cout << img1Path << endl;
    cout << img2Path << endl;
    cout << "*************** INPUT ***************" << endl;
    cout << endl;

    QVector< QVector<Vector3f> > lists = Utils::sift2(img1Path.c_str(),
                                                     img2Path.c_str());

    cout << "Sift completed successfully" << endl;

    Matrix3f F = Utils::calculate_F(lists.at(0), lists.at(1));
    cout << "F: " << endl;
    cout << F << endl;
    Matrix3f K = Utils::build_K(114.873, 0.0130887, 0.0130887, 1936, 1296);
    cout << "K: " << endl;
    cout << K << endl;
    Matrix3f Kl = Utils::build_K(114.873, 0.0130887, 0.0130887, 1936, 1296);
    cout << "Kl: " << endl;
    cout << Kl << endl;
    Matrix3f E = Utils::calculate_E(F, K, Kl);
    cout << "E: " << endl;
    cout << E << endl;
    vector<MatrixXf> Pls = Utils::calculate_P(E);
    QVector<VectorXf> points;
    MatrixXf P(3,4);
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0;
    for (int i = 0; i < Pls.size(); i++)
    {
        stringstream str;
        str << "/home/vagrant/dev/qt-persperctive-distortion-remotion/work-4/MyActions/thai-lion/output" << i << ".obj";
        outputImgPath = str.str();
        points = Utils::get3DPointsByTriangulation(lists.at(0), lists.at(1), P, Pls.at(i));
        Utils::exportObj(outputImgPath, points);
    }


}
