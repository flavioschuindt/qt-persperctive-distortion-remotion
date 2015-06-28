#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


class Utils
{
public:
    Utils();
    static MatrixXd calculateHomographyMatrix(vector<Vector3i> selectedPoints, vector<Vector3d> realWorldPoints);
};

#endif // UTILS_H
