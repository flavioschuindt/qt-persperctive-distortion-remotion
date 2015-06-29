#include "utils.h"

Utils::Utils()
{

}

MatrixXd Utils::calculateHomographyMatrix(vector<Vector3i> selectedPoints, vector<Vector3d> realWorldPoints)
{
    MatrixXd A(8, 8);
    MatrixXd B(8,1);

    for (uint i = 0; i < realWorldPoints.size(); i++)
    {
        double realWorldX = realWorldPoints.at(i).x();
        double realWorldY = realWorldPoints.at(i).y();
        int selectedPointX = selectedPoints.at(i).x();
        int selectedPointY = selectedPoints.at(i).y();

        A.row(i*2) << realWorldX, realWorldY, 1, 0, 0, 0, -realWorldX*selectedPointX, -realWorldY*selectedPointX;
        A.row(i*2 + 1) << 0, 0, 0, realWorldX, realWorldY, 1, -realWorldX*selectedPointY, -realWorldY*selectedPointY;

        B.row(i*2) << selectedPointX;
        B.row(i*2 +1) << selectedPointY;
    }
    A  = A.inverse();

    MatrixXd x = A*B;
    Matrix3d H;

    H << x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), 1;

    return H;
}

void Utils::saveImage(QImage picture, string outputFile)
{
    QImageWriter writer(outputFile.c_str());
    writer.write(picture);
}

void Utils::saveImage(MatrixXi rawData, string outputFile)
{
    QImage* img = new QImage(rawData.cols(), rawData.rows(), QImage::Format_RGB16);
    for (int y = 0; y < img->height(); y++)
    {
        VectorXi a = rawData.row(y);
        memcpy(img->scanLine(y), (void*)&a, img->bytesPerLine());
    }
    QString file = QString::fromUtf8(outputFile.c_str());
    img->save(file);
}

QImage Utils::applyHomography(MatrixXd H, QImage inputImage, int w, int h)
{
    QImage outputImage(w, h, QImage::Format_RGB32);
    MatrixXd x(3,1);
    MatrixXd y(3,1);

    QPainter painter;
    painter.begin(&outputImage);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setBackgroundMode(Qt::TransparentMode);
    QPen pen;

    for (int i=0; i < h; i++)
    {
        for (int j=0; j<w; j++)
        {
            x << i, j, 1;
            y = H*x;
            y << round(y(0,0)/y(2,0)), round(y(1,0)/y(2,0)), 1;

            if (y(0,0) >= 0 && y(0,0) < inputImage.width() && y(1,0) >= 0 && y(1,0) < inputImage.height())
            {
                // Point lies inside the input Image
                QColor clrCurrent( inputImage.pixel( y(0,0), y(1,0) ) );
                pen.setColor(clrCurrent);
                pen.setWidth(5);
                pen.setCapStyle(Qt::RoundCap);
                painter.setPen(pen);
                painter.drawPoint(i, j);

            }
            else
            {
                // Point lies outside the input Image
                QColor clrCurrent(0,0,0);
                pen.setColor(clrCurrent);
                pen.setWidth(5);
                pen.setCapStyle(Qt::RoundCap);
                painter.setPen(pen);
                painter.drawPoint(i, j);
            }
        }
    }
    painter.end();
    return outputImage;
}

