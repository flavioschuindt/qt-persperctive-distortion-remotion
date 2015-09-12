#include "mainwindow.h"

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
    cout << K << endl;
    Kl = Utils::build_K(114.873, 0.0130887, 0.0130887, 1936, 1296);
    cout << "Kl: " << endl;
    cout << Kl << endl;

    R1 <<   0.980106, -0.0199563, 0.197469,
            0.0558328, 0.982476, -0.177828,
            -0.190459, 0.185315, 0.964045;

    cout << "R1: " << endl;
    cout << R1 << endl;

    R2 << 0.914099, -0.0148061, -0.40522,
        -0.0540653, 0.98596, -0.157987,
        0.401869, 0.166324, 0.900465;

    cout << "R2: " << endl;
    cout << R2 << endl;


    t1 << 79.3959, -114.356, -499.541;
    cout << "t1: " << endl;
    cout << t1 << endl;

    t2 << -227.173, -103.559, -460.851;
    cout << "t2: " << endl;
    cout << t2 << endl;


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

    //QVector< QVector<Vector3f> > lists = Utils::sift2(img1Path.c_str(),
                                                     //img2Path.c_str());

    createInitialMatrixes();
    QVector< QVector<Vector3f> > lists = Utils::thaiLionCorrespondences();

    P = K * Rt1;
    Pl = Kl * Rt2;

    QVector<Eigen::Vector3f> points2D_l1, points2D_l2;

    Utils::readPointsFromObj("/home/vagrant/dev/qt-persperctive-distortion-remotion/work-4/MyActions/thai-lion/thai-lion-faces.obj", Points3D, 36000);
    Utils::buildCorrespondenceFrom3DPoints(Points3D, P, Pl, points2D_l1, points2D_l2);

    Matrix3f F = Utils::calculate_F(lists.at(0), lists.at(1), true);
    cout << "F: " << endl;
    cout << F << endl;

    float errorF = Utils::calculateErrorFundamentalMatrix(lists.at(0), lists.at(1), F);
    cout << "Error F: " << endl;
    cout << errorF << endl;

    Matrix3f E = Utils::calculate_E(F, K, Kl);
    cout << "E: " << endl;
    cout << E << endl;
    vector<MatrixXf> Pls = Utils::calculate_P(E);
    QVector<VectorXf> points;
    //MatrixXf P(3,4);
    P = MatrixXf::Identity(3, 4);
    P.block(0, 0, 3, 3) = K;
    cout << "P: " << endl;
    cout << P << endl;
    for (int i = 0; i < Pls.size(); i++)
    {
        stringstream str;
        str << "/home/vagrant/dev/qt-persperctive-distortion-remotion/work-4/MyActions/thai-lion/output" << i << ".obj";
        outputImgPath = str.str();
        Pls.at(i) = Kl * Pls.at(i);
        points = Utils::get3DPointsByTriangulation(points2D_l1, points2D_l2, P, Pls.at(i));
        Utils::exportObj(outputImgPath, points);
    }


}
