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
        string img1Path;
        string img2Path;
        string outputImgPath;
        img1Path = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite2.jpg";
        img2Path = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite3.jpg";
        outputImgPath = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite2-3.jpg";

        QList<Dot *> points = ui->pictureContainer->selectedPoints;
        for( int i=0; i < points.count(); i+=2 )
        {
            pair<Dot *, Dot *> pair(points.at(i), points.at(i+1));
            pairs.push_back(pair);
        }

        H = Utils::dltNormalized(pairs);
        QImage inputImage1 = QImage(img1Path.c_str());
        QImage inputImage2 = QImage(img2Path.c_str());

        vector< std::pair<QImage, Matrix3d> > inputPairs;
        cout << H << endl;
        inputPairs.push_back(std::pair<QImage, Matrix3d>(inputImage1, Matrix<double, 3, 3>::Identity()));
        inputPairs.push_back(std::pair<QImage, Matrix3d>(inputImage2, H));

        QImage outputImage = Utils::panoramic(inputPairs);
        Utils::saveImage(outputImage, outputImgPath);
    }
    else if (METHOD == 1)
    {
        for (int i = 1; i <= 7; i++)
        {
            string img1Path;
            string img2Path;
            string outputImgPath;
            /*if (i == 1)
            {
                img1Path = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite1.jpg";
                img2Path = "/home/fschuind  t/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite2.jpg";
                outputImgPath = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite1-2.jpg";
            }
            else
            {
                string rootPath("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite");
                img1Path = rootPath+Utils::intToString(i-1)+"-"+Utils::intToString(i)+".jpg";
                img2Path = rootPath+Utils::intToString(i+1)+".jpg";
                outputImgPath = rootPath+Utils::intToString(i)+"-"+Utils::intToString(i+1)+".jpg";
            }*/
            img1Path = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite1.jpg";
            img2Path = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite2.jpg";
            outputImgPath = "/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-3/MyActions/yosemite1-2.jpg";

            QImage inputImage1 = QImage(img1Path.c_str());
            QImage inputImage2 = QImage(img2Path.c_str());

            cout << "*************** INPUT ***************" << endl;
            cout << img1Path << endl;
                        cout << img2Path << endl;
            cout << "*************** INPUT ***************" << endl;
            cout << endl;

            /*pairs = Utils::sift(img1Path.c_str(),
                                img2Path.c_str());

            cout << "*************** NUMERO DE PARES DO MATCHER ***************" << endl;
            cout << (int)pairs.size() << endl;
            cout << "*************** NUMERO DE PARES DO MATCHER ***************" << endl;
            cout << endl;

            cout << "H ransac" << endl;
            H = Utils::ransac(pairs, 4, 200, 0.05);
            cout << H << endl;
            cout << "H ransac" << endl;
            cout << endl;*/

            QVector< QVector<Vector3f> > lists = Utils::sift2(img1Path.c_str(),
                                                             img2Path.c_str());

            cout << "H ransac2" << endl;
            Matrix3f Hf = Utils::ransac2(lists.at(0), lists.at(1), 200, 0.05, true, 4);
            H << (double)Hf(0,0), (double)Hf(0,1), (double)Hf(0,2),
                 (double)Hf(1,0), (double)Hf(1,1), (double)Hf(1,2),
                 (double)Hf(2,0), (double)Hf(2,1), (double)Hf(2,2);
            cout << H << endl;
            cout << "H ransac2" << endl;
            cout << endl;

            H = H.inverse().eval();

            vector< std::pair<QImage, Matrix3d> > inputPairs;
            inputPairs.push_back(std::pair<QImage, Matrix3d>(inputImage1, Matrix<double, 3, 3>::Identity()));
            inputPairs.push_back(std::pair<QImage, Matrix3d>(inputImage2, H));

            QImage outputImage = Utils::panoramic(inputPairs);
            Utils::saveImage(outputImage, outputImgPath);
            break;
        }

    }
}
