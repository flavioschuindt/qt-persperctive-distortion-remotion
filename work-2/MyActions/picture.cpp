#include "picture.h"
#include "dot.h"
#include "utils.h"
#include "line.h"

#define METHOD 3

Picture::Picture(QWidget *parent)
    : QLabel(parent)
{
    boardDimensions = new Vector2i(81900, 61300);
}

Picture::~Picture()
{

}

bool Picture::isReadyToProcessUserInputedData(QMouseEvent *e)
{
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
        return false;
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
             return false;
         }
         else
         {
             return true;
         }
    }
}

void Picture::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton) {
         if (METHOD == 1)
         {
              if (isReadyToProcessUserInputedData(e))
              {
                   // Calculate H Matrix
                   MatrixXd H = Utils::calculateHomographyMatrixFromHorizonLine(Utils::getHorizonLine(selectedLines));

                   QImage inputImage = QImage("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-2/MyActions/piso-perspectiva.jpg");
                   QImage outputImage = Utils::applyHomography(H, inputImage, selectedPoints);
                   Utils::saveImage(outputImage, "/home/fschuindt/teste.jpg");
               }
         }
         else if (METHOD == 2)
         {
            if (selectedPoints.size() < 3)
            {
                Dot* dot = new Dot(this);
                dot->show();
                update();
                dot->move(e->x(), e->y());
                selectedPoints.push_back(dot);
                showMessage = true;
            }
            else if (selectedPoints.size() < 4)
            {
                if (showMessage)
                {
                    QMessageBox::information(this, "Próximo Passo", "Agora, escolha um ponto D não colinear com A,B,C.");
                    showMessage = false;
                }
                else
                {
                    Dot* dot = new Dot(this);
                    dot->show();
                    update();
                    dot->move(e->x(), e->y());
                    selectedPoints.push_back(dot);
                }
            }
         }
         else if(METHOD == 3)
         {
             if (isReadyToProcessUserInputedData(e))
             {
                  QMessageBox::information(this, "Próximo Passo", "TODO: IMPLEMENT");
             }
         }

    }

}

