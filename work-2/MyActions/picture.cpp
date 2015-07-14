#include "picture.h"
#include "dot.h"
#include "utils.h"
#include "line.h"

#define METHOD 4

Picture::Picture(QWidget *parent)
    : QLabel(parent)
{
    boardDimensions = new Vector2i(81900, 61300);
}

Picture::~Picture()
{

}

bool Picture::isReadyToProcessUserInputedData(QMouseEvent *e, int totalLines)
{
    if (selectedLines.size() < totalLines)
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
              if (isReadyToProcessUserInputedData(e, 4))
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
             if (isReadyToProcessUserInputedData(e, 4))
             {
                   QList<Line*> firstPair;
                   QList<Line*> secondPair;
                   firstPair.append(selectedLines.at(0));
                   firstPair.append(selectedLines.at(1));
                   secondPair.append(selectedLines.at(2));
                   secondPair.append(selectedLines.at(3));
                   Matrix2d S = Utils::getS(firstPair, secondPair); // S == KKt
                   MatrixXd K = Utils::getUpperTriangularCholesky(S);
                   Matrix3d H = Utils::calculateHomographyMatrixFromCholeskyDecomposition(K);

                   QImage inputImage = QImage("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-2/MyActions/piso-afim-retas-paralelas.jpg");
                   QImage outputImage = Utils::applyHomography(H, inputImage, selectedPoints);
                   Utils::saveImage(outputImage, "/home/fschuindt/teste.jpg");
             }
         }
         else if(METHOD == 4)
         {
             if (isReadyToProcessUserInputedData(e, 10))
             {
                   QList<Line*> firstPair;
                   QList<Line*> secondPair;
                   QList<Line*> thirdPair;
                   QList<Line*> fourthPair;
                   QList<Line*> fifthPair;
                   firstPair.append(selectedLines.at(0));
                   firstPair.append(selectedLines.at(1));
                   secondPair.append(selectedLines.at(2));
                   secondPair.append(selectedLines.at(3));
                   thirdPair.append(selectedLines.at(4));
                   thirdPair.append(selectedLines.at(5));
                   fourthPair.append(selectedLines.at(6));
                   fourthPair.append(selectedLines.at(7));
                   fifthPair.append(selectedLines.at(8));
                   fifthPair.append(selectedLines.at(9));
                   Matrix3d H = Utils::calculateHomographyMatrixFromFiveOrtoghonalLines(firstPair,
                                                                                        secondPair,
                                                                                        thirdPair,
                                                                                        fourthPair,
                                                                                        fifthPair);

                   QImage inputImage = QImage("/home/fschuindt/dev/qt-persperctive-distortion-remotion/work-2/MyActions/piso-perspectiva.jpg");
                   QImage outputImage = Utils::applyHomography(H, inputImage, selectedPoints);
                   Utils::saveImage(outputImage, "/home/fschuindt/teste.jpg");
             }
         }

    }

}

