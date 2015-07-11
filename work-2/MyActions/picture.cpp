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
               }

          }
          else
          {
               if (selectedPoints.size() < 4)
               {
                   Dot* dot = new Dot(this);
                   dot->show();
                   update();
                   dot->move(e->x(), e->y());
                   selectedPoints.push_back(dot);
               }
               else
               {
                   //QMessageBox::information(this, "title", "x");
                   // Calculate H Matrix
                   MatrixXd H = Utils::calculateHomographyMatrixFromHorizonLine(Utils::getHorizonLine(selectedLines));

                   QImage inputImage = QImage("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-2/MyActions/piso-perspectiva.jpg");
                   QImage outputImage = Utils::applyHomography(H, inputImage, selectedPoints);
                   Utils::saveImage(outputImage, "/home/fschuindt/teste.jpg");
               }
          }

    }

}

