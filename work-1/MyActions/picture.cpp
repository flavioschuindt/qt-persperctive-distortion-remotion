#include "picture.h"
#include "dot.h"
#include "utils.h"
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>

Picture::Picture(QWidget *parent)
    : QLabel(parent)
{
    boardDimensions = new Vector2i(81900, 61300);
}

Picture::~Picture()
{

}

void Picture::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton) {
           if (selectedPoints.size() < 8)
           {
               Dot* dot = new Dot(this);
               dot->show();
               update();
               dot->move(e->x(), e->y());
               Vector3i *newPoint = new Vector3i(e->x(), e->y(), 1);
               selectedPoints.push_back(*newPoint);
           }
           else
           {
               double w = (double)boardDimensions->x();
               double h = (double)boardDimensions->y();

               double divisor = w;
               if(w>h)
                divisor = h;

               w = ((double)boardDimensions->x()/divisor);
               h = ((double)boardDimensions->y()/divisor);

               realWorldPoints.push_back(Vector3d(0, 0, 1));
               realWorldPoints.push_back(Vector3d(0, h, 1));
               realWorldPoints.push_back(Vector3d(w, h, 1));
               realWorldPoints.push_back(Vector3d(w, 0, 1));

               // Calculate H Matrix
               MatrixXd H = Utils::calculateHomographyMatrix(selectedPoints, realWorldPoints);

               QImage inputImage = QImage("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-1/MyActions/brahma01.jpg");
               QImage outputImage = Utils::applyHomography(H, inputImage, vector<Vector3i>(selectedPoints.begin()+4, selectedPoints.end()));
               Utils::saveImage(outputImage, "/home/fschuindt/teste.jpg");

           }
    }

}

