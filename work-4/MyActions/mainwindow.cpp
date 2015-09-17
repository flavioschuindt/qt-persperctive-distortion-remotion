#include "mainwindow.h"

#define MANUAL 1

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

void MainWindow::createInitialMatrixes()
{
    K = Utils::build_K(114.873, 0.0130887, 0.0130887, 1936, 1296);
    cout << "K: " << endl;
    cout << K << endl << endl;
    Kl = Utils::build_K(114.873, 0.0130887, 0.0130887, 1936, 1296);
    cout << "Kl: " << endl;
    cout << Kl << endl << endl;

    R1 <<   0.980106, -0.0199563, 0.197469,
            0.0558328, 0.982476, -0.177828,
            -0.190459, 0.185315, 0.964045;

    cout << "R1: " << endl;
    cout << R1 << endl << endl;

    R2 << 0.914099, -0.0148061, -0.40522,
        -0.0540653, 0.98596, -0.157987,
        0.401869, 0.166324, 0.900465;

    cout << "R2: " << endl;
    cout << R2 << endl << endl;


    t1 << 79.3959, -114.356, -499.541;
    cout << "t1: " << endl;
    cout << t1 << endl << endl;

    t2 << -227.173, -103.559, -460.851;
    cout << "t2: " << endl;
    cout << t2 << endl << endl;


    Rt1.resize(3,4);
    Rt2.resize(3,4);
    Rt1.block(0, 0, 3, 3) = R1;
    Rt2.block(0, 0, 3, 3) = R2;
    Rt1.col(3) = -R1 * t1;
    Rt2.col(3) = -R2 * t2;

    P.resize(3,4);
    Pl.resize(3,4);
    P = Rt1;
    Pl = Rt2;
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

    createInitialMatrixes();


    cout << "P matrixes to be used in obtain2DPointsCorrespondenceFrom3DPoints" << endl;
    P = K * Rt1;
    Pl = Kl * Rt2;
    cout << "P: " << endl;
    cout << P << endl << endl;
    cout << "Pl: " << endl;
    cout << Pl << endl << endl;

    QVector<Eigen::Vector3f> points2D_l1, points2D_l2;

    Utils::read3DPointsFromObj("/home/vagrant/dev/qt-persperctive-distortion-remotion/work-4/MyActions/thai-lion/thai-lion-faces.obj", Points3D, 36000);
    Utils::obtain2DPointsCorrespondenceFrom3DPoints(Points3D, P, Pl, points2D_l1, points2D_l2);

    Matrix3f F;
    QVector< QVector<Vector3f> > lists;
    if (MANUAL)
    {
        lists = Utils::thaiLionCorrespondences();
        F = Utils::calculate_F(lists.at(0), lists.at(1), true);
    }
    else
    {
        lists = Utils::sift2(img1Path.c_str(), img2Path.c_str());
        F = Utils::ransac2(lists.at(0), lists.at(1), 1000, 10, false, 8);
        /*F << -1.53484e-07, -1.49698e-06, 0.00180966, // Forcing F to be manual here, it's ok! We have some error in ransac method :(
             1.23965e-06, -5.18379e-08, -0.000459381,
             -0.00117125, 0.000696289, -0.320655;*/
    }

    cout << "F: " << endl;
    cout << F << endl << endl;

    /*float errorF = Utils::calculateErrorFundamentalMatrix(lists.at(0), lists.at(1), F);
    cout << "Error F: " << endl;
    cout << errorF << endl << endl;*/

    Matrix3f E = Utils::calculate_E(F, K, Kl);
    cout << "E: " << endl;
    cout << E << endl << endl;
    vector<MatrixXf> Pls = Utils::calculate_P(E);
    QVector<VectorXf> points;
    //MatrixXf P(3,4);
    P = MatrixXf::Identity(3, 4);
    P.block(0, 0, 3, 3) = K;
    cout << "P matrixes to be used in triangulation" << endl;
    for (int i = 0; i < Pls.size(); i++)
    {
        stringstream str;
        str << "/home/vagrant/dev/qt-persperctive-distortion-remotion/work-4/MyActions/thai-lion/output" << i << ".obj";
        outputImgPath = str.str();
        Pls.at(i) = Kl * Pls.at(i);
        cout << "P: " << endl;
        cout << P << endl << endl;
        cout << "Pl: " << endl;
        cout << Pls.at(i) << endl << endl;
        points = Utils::get3DPointsByTriangulation(points2D_l1, points2D_l2, P, Pls.at(i));
        Utils::saveInObj(outputImgPath, points);
    }


}
