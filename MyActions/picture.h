#ifndef PICTURE_H
#define PICTURE_H

#include <QLabel>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class Picture: public QLabel
{
    Q_OBJECT
public:
    Picture(QWidget *parent = 0);
    ~Picture();
protected:
    void mousePressEvent(QMouseEvent *e);
private:
    vector<Vector3i> selectedPoints;
    vector<Vector3d> realWorldPoints;
    Vector2i *boardDimensions;
};

#endif // PICTURE_H
