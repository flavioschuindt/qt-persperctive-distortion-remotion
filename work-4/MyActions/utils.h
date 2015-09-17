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
#include <QDebug>
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
    static Matrix3d ransac(vector< pair<Dot*,Dot*> > pairs, int numberOfCorrespondences, int n, double threshold);
    static double squaredEuclideanDistance(MatrixXd a, MatrixXd b);
    static string intToString(int a);
    static vector< pair<Dot*,Dot*> > getBestPairs(vector< pair<Dot*,Dot*> > pairs, int n, int numberOfCorrespondences, double threshold);
    static Matrix3d getBestH1(vector< pair<Dot*,Dot*> > pairs, int n, int numberOfCorrespondences, double threshold);
    static cv::Mat QImage2Mat(const QImage &inImage, bool inCloneImageData = true );
    static Matrix3f ransac2(QVector<Vector3f> pA, QVector<Vector3f> pB, double N, double threshold, bool adaptativeSearch, int randomSiz);
    static QVector<int> selectRandomPairs(int numberOfCorrespondences, int size);
    static QVector<int> getRansacInliers(QVector<Vector3f> pA, QVector<Vector3f> pB, Matrix3f H, float threshold);
    static MatrixXf calculate_H(QVector <Vector3f> bp, QVector <Vector3f> rp);
    static float squaredEuclideanDistance(Vector3f a, Vector3f b);
    static float distanceLinePoint(Vector3f l, Vector3f p);
    static QVector< QVector<Vector3f> > sift2(const char *img1Path, const char * img2Path);
    static Matrix3f dlt2(QVector<Vector3f> pointsFirstImage, QVector<Vector3f> pointsSecondImage);
    static Matrix3f getTMatrix2(QVector<Vector3f> points);
    static Matrix3f dltNormalized2(QVector<Vector3f> pointsFirstImage, QVector<Vector3f> pointsSecondImage);
    static Matrix3f gaussNewton(Matrix3f H, QVector<Vector3f> pointsFirstImage, QVector<Vector3f> pointsSecondImage);
    static Matrix3f calculate_F(QVector<Vector3f> pA, QVector<Vector3f> pB, bool  normalizePoints);
    static Matrix3f build_K(float focalmm, float pixelSize_x, float pixelSize_y, float centerPx_x, float centerPx_y);
    static Matrix3f calculate_E(Matrix3f F, Matrix3f K, Matrix3f Kl);
    static vector<MatrixXf> calculate_P(Matrix3f E);
    static QVector<VectorXf> get3DPointsByTriangulation(QVector<Vector3f> pA, QVector<Vector3f> pB, MatrixXf P, MatrixXf Pl);
    static void saveInObj(const string filename, const QVector<VectorXf>  points3D);
    static QVector< QVector<Vector3f> > thaiLionCorrespondences();
    static bool read3DPointsFromObj(const std::string& filename, std::vector<Eigen::Vector3f>& points3D, int max_point_count = INT_MAX);
    static void obtain2DPointsCorrespondenceFrom3DPoints(const std::vector<Eigen::Vector3f>& points3D,
                                        const Eigen::MatrixXf& P,
                                        const Eigen::MatrixXf& Pl,
                                        QVector<Eigen::Vector3f>& points2D_l1,
                                        QVector<Eigen::Vector3f>& points2D_l2);
    static float calculateErrorFundamentalMatrix(QVector<Vector3f> pA, QVector<Vector3f> pB, Matrix3f F);
};

#endif // UTILS_H
