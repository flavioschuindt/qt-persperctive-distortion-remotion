#include "picture.h"
#include "dot.h"
#include "utils.h"
#include "line.h"

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
           if (selectedLines.size() < 4)
           {
               if (selectedPoints.size() < 2)
               {
                   Dot* dot = new Dot(this);
                   dot->show();
                   update();
                   dot->move(e->x(), e->y());
                   //Vector3i *newPoint = new Vector3i(e->x(), e->y(), 1);
                   selectedPoints.push_back(dot);
               }
               else
               {
                  Line* line = new Line(this);
                  line->setA(selectedPoints.at(0));
                  line->setB(selectedPoints.at(1));
                  selectedLines.append(line);
                  selectedPoints.clear();
                  line->show();
                  update();
                  /*QLabel l;
                  QPicture pi;
                  QPainter p(&pi);

                  p.setRenderHint(QPainter::Antialiasing);
                  p.setPen(QPen(Qt::black, 12, Qt::DashDotLine, Qt::RoundCap));
                  p.drawLine(0, 0, 200, 200);
                  p.end(); // Don't forget this line!

                  l.setPicture(pi);
                  l.resize(10,10);
                  l.show();
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
                   Utils::saveImage(outputImage, "/home/fschuindt/teste.jpg");*/

               }

          }
          else
          {
              QMessageBox::information(this, "title", "x");
          }

    }

}

