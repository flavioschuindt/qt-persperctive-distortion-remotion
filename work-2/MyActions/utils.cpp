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
    //A  = A.inverse().eval();

    MatrixXd x = A.fullPivLu().solve(B);
    Matrix3d H;

    H << x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), 1;

    return H.inverse();
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

QColor Utils::interpolate(QImage img, MatrixXd y)
{
    double mappedX;
    double mappedY;
    int mappedXFloor, mappedXCeil, mappedYFloor, mappedYCeil;

    mappedX = y(0,0)/y(2,0);
    mappedY = y(1,0)/y(2,0);

    mappedXFloor = floor(mappedX);
    mappedXCeil = ceil(mappedX);
    mappedYFloor = floor(mappedY);
    mappedYCeil = ceil(mappedY);

    QColor leftTop (img.pixel(mappedXFloor, mappedYFloor));
    QColor rightTop (img.pixel(mappedXCeil, mappedYFloor));
    QColor leftBottom (img.pixel(mappedXFloor, mappedYCeil));
    QColor rightBottom (img.pixel(mappedXCeil, mappedYCeil));

    QColor a ( (((mappedXCeil - y(0,0)) * leftTop.red()) + ( (y(0,0) - mappedXFloor) * rightTop.red())),
               (((mappedXCeil - y(0,0)) * leftTop.green()) + ( (y(0,0) - mappedXFloor) * rightTop.green())),
               (((mappedXCeil - y(0,0)) * leftTop.blue()) + ( (y(0,0) - mappedXFloor) * rightTop.blue()))
               );
    QColor b ( (((mappedXCeil - y(0,0)) * leftBottom.red()) + ( (y(0,0) - mappedXFloor) * rightBottom.red())),
               (((mappedXCeil - y(0,0)) * leftBottom.green()) + ( (y(0,0) - mappedXFloor) * rightBottom.green())),
               (((mappedXCeil - y(0,0)) * leftBottom.blue()) + ( (y(0,0) - mappedXFloor) * rightBottom.blue()))
               );

    QColor c ( ((mappedYCeil - y(1,0)) * a.red() + ( (y(1,0) - mappedYFloor) * b.red() )),
               ((mappedYCeil - y(1,0)) * a.green() + ( (y(1,0) - mappedYFloor) * b.green() )),
               ((mappedYCeil - y(1,0)) * a.blue() + ( (y(1,0) - mappedYFloor) * b.blue() ))
               );

    return c;

}

QImage Utils::applyHomography(MatrixXd H, QImage inputImage, QList<Dot*> region)
{
    MatrixXd x(3,1);
    MatrixXd y(3,1);
    vector<double> x_values;
    vector<double> y_values;

    int height_output = 454;
    int width_output = 604;

    // Calculate output image corners
    x << region.at(0)->x(), region.at(0)->y(), 1;
    y = H*x;
    x_values.push_back(y(0,0)/y(2,0));
    y_values.push_back(y(1,0)/y(2,0));

    x << region.at(1)->x(), region.at(1)->y(), 1;
    y = H*x;
    x_values.push_back(y(0,0)/y(2,0));
    y_values.push_back(y(1,0)/y(2,0));

    x << region.at(2)->x(), region.at(2)->y(), 1;
    y = H*x;
    x_values.push_back(y(0,0)/y(2,0));
    y_values.push_back(y(1,0)/y(2,0));

    x << region.at(3)->x(), region.at(3)->y(), 1;
    y = H*x;
    x_values.push_back(y(0,0)/y(2,0));
    y_values.push_back(y(1,0)/y(2,0));

    double max_x = (*max_element(x_values.begin(), x_values.end()));
    double min_x = (*min_element(x_values.begin(), x_values.end()));
    double max_y = (*max_element(y_values.begin(), y_values.end()));
    double min_y = (*min_element(y_values.begin(), y_values.end()));
    height_output = width_output / ((max_x-min_x)/(max_y-min_y));

    QImage outputImage(width_output, height_output, QImage::Format_ARGB32);
    QPainter painter;
    painter.begin(&outputImage);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setBackgroundMode(Qt::TransparentMode);
    QPen pen;

    H = H.inverse().eval();

    double step = (max_x - min_x) / width_output;

    for (int i=0; i < height_output; i++)
    {
        for (int j=0; j<width_output; j++)
        {
            x << min_x+j*step, min_y+i*step, 1;
            y = H*x;

            y << y(0,0)/y(2,0), y(1,0)/y(2,0), 1;

            if (y(0,0) >= 0 && y(0,0) <= inputImage.width() - 1
                    && y(1,0) >= 0 && y(1,0) <= inputImage.height() - 1)
            {
                // Point lies inside the input Image

                // Do interpolation
                QColor c = interpolate(inputImage, y);
                //QRgb clrCurrent( inputImage.pixel( y(0,0), y(1,0) ) );
                pen.setColor(c);
                pen.setWidth(1);
                pen.setCapStyle(Qt::RoundCap);
                painter.setPen(pen);
                painter.drawPoint(j, i);

            }
            else
            {
                // Point lies outside the input Image
                QColor clrCurrent(0,0,0);
                pen.setColor(clrCurrent);
                pen.setWidth(1);
                pen.setCapStyle(Qt::RoundCap);
                painter.setPen(pen);
                painter.drawPoint(j, i);
            }
        }
    }
    painter.end();
    return outputImage;
}

Vector3d Utils::getHorizonLine(QList<Line*> paralellLines)
{
    Line *firstLine = paralellLines.at(0);
    Line *secondLine = paralellLines.at(1);

    Vector3d a, b;
    a << firstLine->getA()->x(), firstLine->getA()->y(), 1;
    b << firstLine->getB()->x(), firstLine->getB()->y(), 1;
    Vector3d firstLineInHomogeneousCoordinates = a.cross(b);

    a << secondLine->getA()->x(), secondLine->getA()->y(), 1;
    b << secondLine->getB()->x(), secondLine->getB()->y(), 1;
    Vector3d secondLineInHomogeneousCoordinates = a.cross(b);

    Vector3d firstPointHorizonLineInHomogeneousCoordinates = firstLineInHomogeneousCoordinates.cross(secondLineInHomogeneousCoordinates);


    firstLine = paralellLines.at(2);
    secondLine = paralellLines.at(3);

    a << firstLine->getA()->x(), firstLine->getA()->y(), 1;
    b << firstLine->getB()->x(), firstLine->getB()->y(), 1;
    firstLineInHomogeneousCoordinates = a.cross(b);

    a << secondLine->getA()->x(), secondLine->getA()->y(), 1;
    b << secondLine->getB()->x(), secondLine->getB()->y(), 1;
    secondLineInHomogeneousCoordinates = a.cross(b);

    Vector3d secondPointHorizonLineInHomogeneousCoordinates = firstLineInHomogeneousCoordinates.cross(secondLineInHomogeneousCoordinates);

    return firstPointHorizonLineInHomogeneousCoordinates.cross(secondPointHorizonLineInHomogeneousCoordinates);

}

Matrix3d Utils::calculateHomographyMatrixFromHorizonLine(Vector3d horizonLine)
{
    Matrix3d H;
    H << 1, 0, 0, 0, 1, 0, horizonLine(0), horizonLine(1), horizonLine(2);
    return H;
}

Vector3d Utils::getLineInHomogeneousCoordinates(Line *line)
{
    Vector3d a, b;
    a << line->getA()->x(), line->getA()->y(), 1;
    b << line->getB()->x(), line->getB()->y(), 1;
    return a.cross(b);
}

Matrix2d Utils::getS(QList<Line*> firstOrtoghonalLines, QList<Line*> secondOrthogonalLine)
{
    Matrix2d S;

    Vector3d l1 = getLineInHomogeneousCoordinates(firstOrtoghonalLines.at(0));
    Vector3d m1 = getLineInHomogeneousCoordinates(firstOrtoghonalLines.at(1));
    Vector3d l2 = getLineInHomogeneousCoordinates(secondOrthogonalLine.at(0));
    Vector3d m2 = getLineInHomogeneousCoordinates(secondOrthogonalLine.at(1));

    // A * x = b.
    Matrix2d A;
    MatrixXd B(2,1);

    A << l1(0)*m1(0), l1(0)*m1(1)+l1(1)*m1(0), l2(0)*m2(0), l2(0)*m2(1)+l2(1)*m2(0);
    B << -l1(1)*m1(1), -l2(1)*m2(1);

    MatrixXd x = A.fullPivLu().solve(B);

    S << x(0,0), x(1,0), x(1,0), 1;

    return S;
}

MatrixXd Utils::getUpperTriangularCholesky(MatrixXd KKt)
{
    LLT<MatrixXd> llt(KKt);
    MatrixXd K = llt.matrixU();

    return K;

}

Matrix3d Utils::calculateHomographyMatrixFromCholeskyDecomposition(MatrixXd K)
{
    Matrix3d H;
    H << K(0,0), K(0,1), 0, K(1,0), K(1,1), 0, 0, 0, 1;
    return H;
}

