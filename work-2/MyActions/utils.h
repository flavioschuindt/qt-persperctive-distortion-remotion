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

#include "line.h"

using namespace Eigen;
using namespace std;


class Utils
{
public:
    Utils();
    static MatrixXd calculateHomographyMatrix(vector<Vector3i> selectedPoints, vector<Vector3d> realWorldPoints);
    static void saveImage(MatrixXi rawData, string outputFile);
    static void saveImage(QImage picture, string outputFile);
    static QImage applyHomography(MatrixXd H, QImage inputImage, QList<Dot*> region);
    static QColor interpolate(QImage img, MatrixXd y);
    static Vector3d getHorizonLine(QList<Line*> paralellLines);
    static Matrix3d calculateHomographyMatrixFromHorizonLine(Vector3d horizonLine);
    static Matrix2d getS(QList<Line*> firstOrtoghonalLines, QList<Line*> secondOrthogonalLine);
    static MatrixXd calculateHomographyMatrixFromFiveOrtoghonalLines(QList<Line*> firstOrtoghonalLines, QList<Line*> secondOrthogonalLines,
                         QList<Line*> thirdOrthogonalLines, QList<Line*> fourthOrthogonalLines,
                         QList<Line*> fifthOrthogonalLines);
    static Vector3d getLineInHomogeneousCoordinates(Line *line);
    static MatrixXd getUpperTriangularCholesky(MatrixXd K);
    static Matrix3d calculateHomographyMatrixFromCholeskyDecomposition(MatrixXd K);
};

#endif // UTILS_H
