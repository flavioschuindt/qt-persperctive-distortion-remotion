#include "picture.h"
#include "dot.h"
#include "utils.h"
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>

Picture::Picture(QWidget *parent)
    : QLabel(parent)
{
    aspectRatio = 1.3;
    boardDimensions = new Vector2i(819, 613);
}

Picture::~Picture()
{

}

void Picture::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton) {
           if (selectedPoints.size() < 4)
           {
               Dot* dot = new Dot(this);
               dot->show();
               update();
               dot->move(e->x(), e->y());
               Vector3i *newPoint = new Vector3i(e->x(), e->y(), 1);
               selectedPoints.push_back(*newPoint);
               //Vector3i x = selectedPoints.at(selectedPoints.size()-1);
               //QMessageBox::information(this, "title", QString::number(x.x()) + " - " + QString::number(x.y()));
           }
           else
           {
               /*for (std::vector<Vector3i>::iterator it = selectedPoints.begin() ; it != selectedPoints.end(); ++it)
               {
                   Vector3i selectedPoint = *it;

                   int x = selectedPoint.x()/(double)pictureResolution->x()*aspectRatio;
                   int y = selectedPoint.y()/(double)pictureResolution->y();
                   Vector3i *realWorldPoint = new Vector3i(x, y, 1);
                   realWorldPoints.push_back(*realWorldPoint);
                   //QMessageBox::information(this, "title", QString::number(selectedPoint.x()) + " - " + QString::number(selectedPoint.y()));
               }*/

               double w = (double)boardDimensions->x();
               double h = (double)boardDimensions->y();

               double divisor = w;
               if(w>h)
                divisor = h;

               double scaleFactor = 1.0;

               w = ((double)boardDimensions->x()/divisor) * scaleFactor;
               h = ((double)boardDimensions->y()/divisor) * scaleFactor;

               realWorldPoints.push_back(Vector3d(0, 0, 1));
               realWorldPoints.push_back(Vector3d(0, h, 1));
               realWorldPoints.push_back(Vector3d(w, h, 1));
               realWorldPoints.push_back(Vector3d(w, 0, 1));

               // Calculate H Matrix

               MatrixXd H = Utils::calculateHomographyMatrix(selectedPoints, realWorldPoints);
               //H = Matrix<double, 3, 3>::Identity();

               QImage inputImage = QImage("/home/fschuindt/dev/qt-persperctive-distortion-remotion/MyActions/brahma01.jpg");
               QImage outputImage = Utils::applyHomography(H, inputImage);
               Utils::saveImage(outputImage, "/home/fschuindt/teste.jpg");

           }
    }

}

