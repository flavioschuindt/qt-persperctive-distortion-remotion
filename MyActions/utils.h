#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <Eigen/Dense>
#include <QImage>
#include <QString>
#include <QColor>
#include <QSize>
#include <QPainter>
#include <QImageWriter>
#include <iostream>
#include <math.h>

using namespace Eigen;
using namespace std;


class Utils
{
public:
    Utils();
    static MatrixXd calculateHomographyMatrix(vector<Vector3i> selectedPoints, vector<Vector3d> realWorldPoints);
    static void saveImage(MatrixXi rawData, string outputFile);
    static void saveImage(QImage picture, string outputFile);
    static QImage applyHomography(MatrixXd H, QImage inputImage);
};

#endif // UTILS_H
