#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Dense>
#include <QImage>
#include <QString>
#include <QColor>
#include <QSize>
#include <QPainter>
#include <QImageWriter>
#include <iostream>
#include <math.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include "line.h"

struct bounds{
    double top;
    double left;
    double right;
    double bottom;
};

using namespace Eigen;
using namespace std;
using namespace cv;


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
    static Matrix3d dlt(vector< pair<Dot*,Dot*> > pairs);
    static Matrix3d dltNormalized(vector< pair<Dot*,Dot*> > pairs);
    static Matrix3d getTMatrix(vector<Dot *> points);
    static QImage panoramic(vector< std::pair<QImage, Matrix3d> > pairs);
    static bounds getBounds(vector< std::pair<QImage, Matrix3d> > imgs);
    static vector< pair<Dot*,Dot*> > surf(const char * img1, const char * img2);
    static vector< pair<Dot*,Dot*> > sift(const char *img1Path, const char * img2Path);
    static Matrix3d ransac(vector< pair<Dot*,Dot*> > pairs, int numberOfCorrespondences, int n);
    static double squaredEuclideanDistance(MatrixXd a, MatrixXd b);
    static string intToString(int a);
    static vector< pair<Dot*,Dot*> > getBestPairs(vector< pair<Dot*,Dot*> > pairs, int n, int numberOfCorrespondences);
};

#endif // UTILS_H
