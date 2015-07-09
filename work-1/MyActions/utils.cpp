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

QImage Utils::applyHomography(MatrixXd H, QImage inputImage, vector<Vector3i> region)
{
    MatrixXd x(3,1);
    MatrixXd y(3,1);
    vector<double> x_values;
    vector<double> y_values;

    int width_input = inputImage.width();
    int height_input = inputImage.height();
    int height_output = 0;
    int width_output = 960;

    // Calculate output image corners
    x << region.at(0)(0), region.at(0)(1), 1;
    y = H*x;
    x_values.push_back(y(0,0)/y(2,0));
    y_values.push_back(y(1,0)/y(2,0));

    x << region.at(1)(0), region.at(1)(1), 1;
    y = H*x;
    x_values.push_back(y(0,0)/y(2,0));
    y_values.push_back(y(1,0)/y(2,0));

    x << region.at(2)(0), region.at(2)(1), 1;
    y = H*x;
    x_values.push_back(y(0,0)/y(2,0));
    y_values.push_back(y(1,0)/y(2,0));

    x << region.at(3)(0), region.at(3)(1), 1;
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

