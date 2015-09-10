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
    height_output = (width_output / ((max_x-min_x)/(max_y-min_y)));

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

Matrix2d Utils::getS(QList<Line*> firstOrtoghonalLines, QList<Line*> secondOrthogonalLines)
{
    Matrix2d S;

    Vector3d l1 = getLineInHomogeneousCoordinates(firstOrtoghonalLines.at(0));
    Vector3d m1 = getLineInHomogeneousCoordinates(firstOrtoghonalLines.at(1));
    Vector3d l2 = getLineInHomogeneousCoordinates(secondOrthogonalLines.at(0));
    Vector3d m2 = getLineInHomogeneousCoordinates(secondOrthogonalLines.at(1));

    // A * x = b.
    Matrix2d A;
    MatrixXd B(2,1);

    A << l1(0)*m1(0), l1(0)*m1(1)+l1(1)*m1(0), l2(0)*m2(0), l2(0)*m2(1)+l2(1)*m2(0);
    B << -l1(1)*m1(1), -l2(1)*m2(1);

    MatrixXd x = A.fullPivLu().solve(B);

    S << x(0,0), x(1,0), x(1,0), 1;

    return S;
}

MatrixXd Utils::calculateHomographyMatrixFromFiveOrtoghonalLines(QList<Line*> firstOrtoghonalLines, QList<Line*> secondOrthogonalLines,
                     QList<Line*> thirdOrthogonalLines, QList<Line*> fourthOrthogonalLines,
                     QList<Line*> fifthOrthogonalLines)
{
    // A * x = b.
    MatrixXd A(6, 6);
    MatrixXd b(6, 1);
    MatrixXd x;

    Vector3d l1 = getLineInHomogeneousCoordinates(firstOrtoghonalLines.at(0));
    l1 /= l1(2);
    Vector3d m1 = getLineInHomogeneousCoordinates(firstOrtoghonalLines.at(1));
    m1 /= m1(2);
    Vector3d l2 = getLineInHomogeneousCoordinates(secondOrthogonalLines.at(0));
    l2 /= l2(2);
    Vector3d m2 = getLineInHomogeneousCoordinates(secondOrthogonalLines.at(1));
    m2 /= m2(2);
    Vector3d l3 = getLineInHomogeneousCoordinates(thirdOrthogonalLines.at(0));
    l3 /= l3(2);
    Vector3d m3 = getLineInHomogeneousCoordinates(thirdOrthogonalLines.at(1));
    m3 /= m3(2);
    Vector3d l4 = getLineInHomogeneousCoordinates(fourthOrthogonalLines.at(0));
    l4 /= l4(2);
    Vector3d m4 = getLineInHomogeneousCoordinates(fourthOrthogonalLines.at(1));
    m4 /= m4(2);
    Vector3d l5 = getLineInHomogeneousCoordinates(fifthOrthogonalLines.at(0));
    l5 /= l5(2);
    Vector3d m5 = getLineInHomogeneousCoordinates(fifthOrthogonalLines.at(1));
    m5 /= m5(2);

    //b << //-l1(1)*m1(1), -l2(1)*m2(1), -l3(1)*m3(1), -l4(1)*m4(1), -l5(1)*m5(1), 0.0;
    A << l1(0)*m1(0), (l1(0)*m1(1)+l1(1)*m1(0))/2, l1(1)*m1(1), (l1(0)*m1(2)+l1(2)*m1(0))/2, (l1(1)*m1(2)+l1(2)*m1(1))/2, l1(2)*m1(2),
         l2(0)*m2(0), (l2(0)*m2(1)+l2(1)*m2(0))/2, l2(1)*m2(1), (l2(0)*m2(2)+l2(2)*m2(0))/2, (l2(1)*m2(2)+l2(2)*m2(1))/2, l2(2)*m2(2),
         l3(0)*m3(0), (l3(0)*m3(1)+l3(1)*m3(0))/2, l3(1)*m3(1), (l3(0)*m3(2)+l3(2)*m3(0))/2, (l3(1)*m3(2)+l3(2)*m3(1))/2, l3(2)*m3(2),
         l4(0)*m4(0), (l4(0)*m4(1)+l4(1)*m4(0))/2, l4(1)*m4(1), (l4(0)*m4(2)+l4(2)*m4(0))/2, (l4(1)*m4(2)+l4(2)*m4(1))/2, l4(2)*m4(2),
         l5(0)*m5(0), (l5(0)*m5(1)+l5(1)*m5(0))/2, l5(1)*m5(1), (l5(0)*m5(2)+l5(2)*m5(0))/2, (l5(1)*m5(2)+l5(2)*m5(1))/2, l5(2)*m5(2),
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

   FullPivLU<MatrixXd> lu_decomp(A);
   x = lu_decomp.kernel();

   x /= x(5);

   Matrix3d C;
   C << x(0), x(1)/2, x(3)/2,
        x(1)/2, x(2), x(4)/2,
        x(3)/2, x(4)/2, 1;

   Matrix2d kkt;
   kkt << C(0,0), C(0,1),
          C(1,0), C(1,1);

   MatrixXd vKKt(1,2);
   vKKt << C(2,0), C(2,1);

   MatrixXd V(1,2);
   V = vKKt * kkt.inverse();

   LDLT<MatrixXd> ldlt(kkt);
   MatrixXd U = ldlt.matrixU();

   MatrixXd J (3,3);
   J << U(0,0), U(0,1),0, U(1,0), U(1,1),0, V(0), V(1), 1;

   return J;
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

Matrix3d Utils::dlt(vector< pair<Dot*,Dot*> > pairs)
{
    MatrixXd A(pairs.size()*2, 9);

    for(std::vector<int>::size_type i = 0; i < pairs.size(); i++) {
        pair<Dot*,Dot*> pair = pairs.at(i);
        Dot* firstPoint = new Dot();
        firstPoint->move(pair.first->x(), pair.first->y());
        Dot* secondPoint = new Dot();
        secondPoint->move(pair.second->x(), pair.second->y());

        A.row(i * 2) << 0, 0, 0, -firstPoint->x(), -firstPoint->y(), -1, secondPoint->y()*firstPoint->x(), secondPoint->y()*firstPoint->y(), secondPoint->y();
        A.row(i * 2 + 1) << firstPoint->x(), firstPoint->y(), 1, 0, 0, 0, -secondPoint->x()*firstPoint->x(), -secondPoint->x()*firstPoint->y(), -secondPoint->x();

    }

    JacobiSVD<MatrixXd> SVD(A, Eigen::ComputeThinV);
    VectorXd h = SVD.matrixV().col(SVD.matrixV().cols() - 1);
    Matrix3d H;
    H <<    h(0), h(1), h(2),
            h(3), h(4), h(5),
            h(6), h(7), h(8);
    return H;
}

Matrix3d Utils::getTMatrix(vector<Dot *> points)
{
    double u_avg = 0;
    double v_avg = 0;
    int u_sum = 0;
    int v_sum = 0;

    for(std::vector<int>::size_type i = 0; i < points.size(); i++) {
        Dot * point = points.at(i);
        u_sum += point->x();
        v_sum += point->y();
    }

    u_avg = u_sum / (double)points.size();
    v_avg = v_sum / (double)points.size();


    double sum = 0;
    double s = 0;
    for(std::vector<int>::size_type i = 0; i < points.size(); i++) {
        Dot * point = points.at(i);
        sum += sqrt(pow(((double)point->x() - u_avg), 2) + pow(((double)point->y() - v_avg), 2));
    }
    s = (sqrt(2) * (double)points.size()) / sum;

    Matrix3d T;
    T << s, 0, -s*u_avg,
         0, s, -s*v_avg,
         0, 0, 1;
    return T;
}

Matrix3d Utils::dltNormalized(vector< pair<Dot*,Dot*> > pairs)
{

    vector<Dot *> pointsFirstImage;
    vector<Dot *> pointsSecondImage;

    for(std::vector<int>::size_type i = 0; i < pairs.size(); i++) {
        pair<Dot*,Dot*> pair = pairs.at(i);
        Dot* firstPoint = new Dot();
        firstPoint->move(pair.first->x(), pair.first->y());
        Dot* secondPoint = new Dot();
        secondPoint->move(pair.second->x(), pair.second->y());
        pointsFirstImage.push_back(firstPoint);
        pointsSecondImage.push_back(secondPoint);
    }

    Matrix3d T = getTMatrix(pointsFirstImage);
    Matrix3d T2 = getTMatrix(pointsSecondImage);

    Vector3d p1, p2;
    vector< pair<Dot*,Dot*> > normalizedPairs;

    for(std::vector<int>::size_type i = 0; i < pairs.size(); i++) {
        pair<Dot*,Dot*> pair = pairs.at(i);
        Dot* firstPoint = new Dot();
        firstPoint->move(pair.first->x(), pair.first->y());
        Dot* secondPoint = new Dot();
        secondPoint->move(pair.second->x(), pair.second->y());

        p1 << firstPoint->x(), firstPoint->y(), 1;
        p2 << secondPoint->x(), secondPoint->y(), 1;

        Vector3d res = T * p1;
        firstPoint->move(res(0), res(1));
        res = T2 * p2;
        secondPoint->move(res(0), res(1));

        std::pair<Dot*,Dot*> normalizedPair(firstPoint, secondPoint);
        normalizedPairs.push_back(normalizedPair);
    }

    Matrix3d Hn = dlt(normalizedPairs);
    Matrix3d H = T.inverse().eval() * Hn * T2;

    return H;

}

bounds Utils::getBounds(vector< std::pair<QImage, Matrix3d> > pairs)
{
    bounds bound;
    MatrixXd x(3,1);
    MatrixXd y(3,1);
    Matrix3d H;
    vector<double> x_values;
    vector<double> y_values;
    QImage img;

    // Calculate output image corners
    for (std::vector<int>::size_type i = 0; i < pairs.size(); i++)
    {
        std::pair<QImage, Matrix3d> pair = pairs.at(i);
        img = pair.first;
        H = pair.second;
        x << 0, 0, 1;
        y = H*x;
        x_values.push_back(y(0,0)/y(2,0));
        y_values.push_back(y(1,0)/y(2,0));

        x << img.width(), 0, 1;
        y = H*x;
        x_values.push_back(y(0,0)/y(2,0));
        y_values.push_back(y(1,0)/y(2,0));

        x << img.width(), img.height(), 1;
        y = H*x;
        x_values.push_back(y(0,0)/y(2,0));
        y_values.push_back(y(1,0)/y(2,0));

        x << 0, img.height(), 1;
        y = H*x;
        x_values.push_back(y(0,0)/y(2,0));
        y_values.push_back(y(1,0)/y(2,0));
    }

    bound.right = (*max_element(x_values.begin(), x_values.end()));
    bound.left = (*min_element(x_values.begin(), x_values.end()));
    bound.bottom = (*max_element(y_values.begin(), y_values.end()));
    bound.top = (*min_element(y_values.begin(), y_values.end()));

    return bound;
}

QImage Utils::panoramic(vector< std::pair<QImage, Matrix3d> > pairs)
{
    bounds bound = getBounds(pairs);
    int width_output = 1152;
    int height_output = (width_output / ((bound.right-bound.left)/(bound.bottom-bound.top)));
    MatrixXd x(3,1);
    MatrixXd y(3,1);

    QImage newImage = QImage(width_output,  height_output, QImage::Format_ARGB32);

    QPainter painter;
    painter.begin(&newImage);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setBackgroundMode(Qt::TransparentMode);
    QPen pen;

    double step = (bound.right-bound.left) / width_output;

    for (std::vector<int>::size_type i = 0; i < pairs.size(); i++)
    {
        std::pair<QImage, Matrix3d> pair = pairs.at(i);
        QImage inputImage = pair.first;
        Matrix3d H = pair.second;
        H = H.inverse().eval();
        for (int i=0; i < height_output; i++)
        {
            for (int j=0; j<width_output; j++)
            {
                x << bound.left+j*step, bound.top+i*step, 1;
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
                /*else
                {
                    // Point lies outside the input Image
                    QColor clrCurrent(0,0,0);
                    pen.setColor(clrCurrent);
                    pen.setWidth(1);
                    pen.setCapStyle(Qt::RoundCap);
                    painter.setPen(pen);
                    painter.drawPoint(j, i);
                }*/
            }
        }
    }
    painter.end();
    return newImage;
}

vector< pair<Dot*,Dot*> > Utils::sift(const char *img1Path, const char * img2Path)
{
    vector< pair<Dot*, Dot*> > goodPairs;

    QImage inputImage1 = QImage(img1Path);
    QImage inputImage2 = QImage(img2Path);

    //Mat img_1 = imread( img1Path, CV_LOAD_IMAGE_GRAYSCALE );
    //Mat img_2 = imread( img2Path, CV_LOAD_IMAGE_GRAYSCALE );

    Mat img_1 = Utils::QImage2Mat(inputImage1);
    Mat img_2 = Utils::QImage2Mat(inputImage2);

    if( !img_1.data || !img_2.data )
        return goodPairs;

      //-- Step 1: Detect the keypoints using SIFT Detector

      SiftFeatureDetector detector(20000);

      std::vector<KeyPoint> keypoints_1, keypoints_2;

      detector.detect( img_1, keypoints_1 );
      detector.detect( img_2, keypoints_2 );

      //-- Step 2: Calculate descriptors (feature vectors)
      SiftDescriptorExtractor extractor;

      Mat descriptors_1, descriptors_2;

      extractor.compute( img_1, keypoints_1, descriptors_1 );
      extractor.compute( img_2, keypoints_2, descriptors_2 );

      //-- Step 3: Matching descriptor vectors using FLANN matcher
      FlannBasedMatcher matcher;
      std::vector< DMatch > matches;
      matcher.match( descriptors_1, descriptors_2, matches );

      double max_dist = 0; double min_dist = 100;

      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < (int)matches.size(); i++ )
      {
        double dist = matches[1].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }

      //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
      //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
      //-- small)
      //-- PS.- radiusMatch can also be used here.
      std::vector< DMatch > good_matches;

      for( int i = 0; i < (int)matches.size(); i++ )
      {
          if( matches[i].distance <= max(2*min_dist, 0.02) )
            good_matches.push_back( matches[i]);
      }

      //-- Draw only "good" matches
      /*Mat img_matches;
      drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

      //-- Show detected matches
      imshow( "Good Matches", img_matches );*/

      for( int i = 0; i < (int)good_matches.size(); i++ )
      {
          //printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
          Point2i point1 = keypoints_1[good_matches[i].queryIdx].pt;
          Point2i point2 = keypoints_2[good_matches[i].trainIdx].pt;
          Dot* dot1 = new Dot();
          dot1->move(point1.x, point1.y);
          Dot* dot2 = new Dot();
          dot2->move(point2.x, point2.y);

          pair<Dot *, Dot *> pair(dot1, dot2);
          goodPairs.push_back(pair);
      }

      //waitKey(0);

      return goodPairs;
}

vector< pair<Dot*,Dot*> > Utils::surf(const char *img1Path, const char * img2Path)
{
    vector< pair<Dot*, Dot*> > goodPairs;

    Mat img_1 = imread( img1Path, CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_2 = imread( img2Path, CV_LOAD_IMAGE_GRAYSCALE );

    if( !img_1.data || !img_2.data )
        return goodPairs;

      //-- Step 1: Detect the keypoints using SURF Detector
      int minHessian = 5100;

      SurfFeatureDetector detector( minHessian );

      std::vector<KeyPoint> keypoints_1, keypoints_2;

      detector.detect( img_1, keypoints_1 );
      detector.detect( img_2, keypoints_2 );

      //-- Step 2: Calculate descriptors (feature vectors)
      SurfDescriptorExtractor extractor;

      Mat descriptors_1, descriptors_2;

      extractor.compute( img_1, keypoints_1, descriptors_1 );
      extractor.compute( img_2, keypoints_2, descriptors_2 );

      //-- Step 3: Matching descriptor vectors using FLANN matcher
      FlannBasedMatcher matcher;
      std::vector< DMatch > matches;
      matcher.match( descriptors_1, descriptors_2, matches );

      double max_dist = 0; double min_dist = 100;

      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptors_1.rows; i++ )
      {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }

      //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
      //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
      //-- small)
      //-- PS.- radiusMatch can also be used here.
      std::vector< DMatch > good_matches;

      for( int i = 0; i < descriptors_1.rows; i++ )
      {
          if( matches[i].distance <= max(2*min_dist, 0.02) )
            good_matches.push_back( matches[i]);
      }

      //-- Draw only "good" matches
      /*Mat img_matches;
      drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

      //-- Show detected matches
      imshow( "Good Matches", img_matches );*/

      for( int i = 0; i < (int)good_matches.size(); i++ )
      {
          //printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
          Point2i point1 = keypoints_1[good_matches[i].queryIdx].pt;
          Point2i point2 = keypoints_2[good_matches[i].trainIdx].pt;
          Dot* dot1 = new Dot();
          dot1->move(point1.x, point1.y);
          Dot* dot2 = new Dot();
          dot2->move(point2.x, point2.y);

          pair<Dot *, Dot *> pair(dot1, dot2);
          goodPairs.push_back(pair);
      }

      //waitKey(0);

      return goodPairs;
}

vector< pair<Dot*,Dot*> > Utils::getBestPairs(vector< pair<Dot*,Dot*> > pairs, int n, int numberOfCorrespondences, double threshold)
{
    srand (time(NULL));
    int maxInliers = 0;
    vector< pair<Dot*,Dot*> > bestInliers;
    vector< pair<Dot*,Dot*> > bestInliersInCurrentIteration;
    vector< pair<Dot*,Dot*> > randomPairs;
    //double N = 588;
    double Ni = 0;
    double w = 1; // Inliers likelihood
    double e = 0; // Outliers likelihood
    double p = 0.99;
    bool adaptativeSearch = true;
    int ransacCounter = 0;

    while(ransacCounter < n){
    //for (int i = 0; i < n; i++){

        bestInliersInCurrentIteration.clear();
        int inliers = 0;
        // get random correspondences
        randomPairs.clear();
        QVector<Vector3f> l1, l2;
        for (int j = 0; j < numberOfCorrespondences; j++)
        {
            int correspondence = rand() % pairs.size();

            pair <Dot*, Dot*> pair = pairs.at(correspondence);
            Dot* pointInFirstImage = new Dot();
            Dot* pointInSecondImage = new Dot();
            pointInFirstImage->move(pair.first->x(), pair.first->y());
            pointInSecondImage->move(pair.second->x(), pair.second->y());

            std::pair<Dot *, Dot *> randomPair(pointInFirstImage, pointInSecondImage);
            randomPairs.push_back(randomPair);

            Vector3f a, b;
            a << pointInFirstImage->x(), pointInFirstImage->y(), 1;
            b << pointInSecondImage->x(), pointInSecondImage->y(), 1;
            l1.push_back(a);
            l2.push_back(b);
        }

        // do DLT with these random pairs
        /*Matrix3d H1 = dltNormalized(randomPairs);
        H1 = H1.inverse().eval();*/

        Matrix3f H1f = calculate_H(l1, l2);
        Matrix3d H1;
        H1 << (double)H1f(0,0), (double)H1f(0,1), (double)H1f(0,2),
             (double)H1f(1,0), (double)H1f(1,1), (double)H1f(1,2),
             (double)H1f(2,0), (double)H1f(2,1), (double)H1f(2,2);
        // Apply H in all points in the first image and calculate error between z and y
        MatrixXd y(3,1);
        MatrixXd x(3,1);
        MatrixXd z(3,1);
        for (int k = 0; k < (int)pairs.size(); k++)
        {
            Dot *pointInFirstImage = new Dot();
            Dot *pointInSecondImage = new Dot();
            pair <Dot*, Dot*> pair = pairs.at(k);
            pointInFirstImage->move(pair.first->x(), pair.first->y());
            pointInSecondImage->move(pair.second->x(), pair.second->y());
            x << pointInFirstImage->x(), pointInFirstImage->y(),1;
            y = H1*x;
            y << y(0,0)/y(2,0), y(1,0)/y(2,0), 1;
            z << pointInSecondImage->x(), pointInSecondImage->y(),1;
            double distance = squaredEuclideanDistance(z, y);

            if ( distance < threshold)
            {
                inliers++;
                std::pair <Dot*, Dot*> inlier(pointInFirstImage, pointInSecondImage);
                bestInliersInCurrentIteration.push_back(inlier);
            }
        }
        if (inliers > maxInliers)
        {
            maxInliers = inliers;
            bestInliers = bestInliersInCurrentIteration;
        }

        if(adaptativeSearch){
            e = 1 - ((float)inliers/pairs.size());
            if(e < w){
                w = e;
                Ni = 72;
                if(Ni < n){
                     adaptativeSearch = false;
                }else{
                    n = Ni;
                }
            }
        }
        ransacCounter++;
        if(ransacCounter >= 1000)
            break;

    }
    return bestInliers;
}

Matrix3d Utils::getBestH1(vector< pair<Dot*,Dot*> > pairs, int n, int numberOfCorrespondences, double threshold)
{
    srand (time(NULL));
    int maxInliers = 0;
    vector< pair<Dot*,Dot*> > bestInliers;
    vector< pair<Dot*,Dot*> > bestInliersInCurrentIteration;
    vector< pair<Dot*,Dot*> > randomPairs;
    Matrix3d bestH1;
    for (int i = 0; i < n; i++)
    {
        bestInliersInCurrentIteration.clear();
        int inliers = 0;
        // get random correspondences
        randomPairs.clear();
        for (int j = 0; j < numberOfCorrespondences; j++)
        {
            int correspondence = rand() % pairs.size();

            pair <Dot*, Dot*> pair = pairs.at(correspondence);
            Dot* pointInFirstImage = new Dot();
            Dot* pointInSecondImage = new Dot();
            pointInFirstImage->move(pair.first->x(), pair.first->y());
            pointInSecondImage->move(pair.second->x(), pair.second->y());

            std::pair<Dot *, Dot *> randomPair(pointInFirstImage, pointInSecondImage);
            randomPairs.push_back(randomPair);
        }

        // do DLT with these random pairs
        Matrix3d H1 = dltNormalized(randomPairs);

        // Apply H in all points in the first image and calculate error between z and y
        MatrixXd y(3,1);
        MatrixXd x(3,1);
        MatrixXd z(3,1);

        for (int k = 0; k < (int)pairs.size(); k++)
        {
            Dot *pointInFirstImage = new Dot();
            Dot *pointInSecondImage = new Dot();
            pair <Dot*, Dot*> pair = pairs.at(k);
            pointInFirstImage->move(pair.first->x(), pair.first->y());
            pointInSecondImage->move(pair.second->x(), pair.second->y());
            x << pointInFirstImage->x(), pointInFirstImage->y(),1;
            y = H1*x;
            y << y(0,0)/y(2,0), y(1,0)/y(2,0), 1;
            z << pointInSecondImage->x(), pointInSecondImage->y(),1;
            if (squaredEuclideanDistance(z, y) < threshold)
            {
                inliers++;
                std::pair <Dot*, Dot*> inlier(pointInFirstImage, pointInSecondImage);
                bestInliersInCurrentIteration.push_back(inlier);
            }
        }
        if (inliers > maxInliers)
        {
            maxInliers = inliers;
            bestInliers = bestInliersInCurrentIteration;
            bestH1 = H1;
        }

    }
    return bestH1;
}

Matrix3d Utils::ransac(vector< pair<Dot*,Dot*> > pairs, int numberOfCorrespondences, int n, double threshold)
{
    vector< pair<Dot*,Dot*> > bestInliers = getBestPairs(pairs, n, numberOfCorrespondences, threshold);
    cout << "=========" << endl;
    cout << (int)pairs.size() << endl;
    cout << (int)bestInliers.size() << endl;
    Matrix3d H = dltNormalized(bestInliers);
    return H;

}

double Utils::squaredEuclideanDistance(MatrixXd a, MatrixXd b)
{
   // a e b are column vectors
   double distance = 0;
   for (int i = 0; i < a.rows(); i++)
        distance += pow( (a(i,0) - b(i,0)), 2);
   return distance;
}

string Utils::intToString(int a)
{
    std::string s;
    std::stringstream out;
    out << a;
    s = out.str();
    return s;
}

cv::Mat Utils::QImage2Mat(const QImage &inImage, bool inCloneImageData)
{
    switch ( inImage.format() )
    {
       // 8-bit, 4 channel
       case QImage::Format_RGB32:
       {
          cv::Mat  mat( inImage.height(), inImage.width(), CV_8UC4, const_cast<uchar*>(inImage.bits()), inImage.bytesPerLine() );
          return (inCloneImageData ? mat.clone() : mat);
       }
       // 8-bit, 3 channel
       case QImage::Format_RGB888:
       {
          if ( !inCloneImageData )
             qWarning() << "ASM::QImageToCvMat() - Conversion requires cloning since we use a temporary QImage";

          QImage   swapped = inImage.rgbSwapped();
          return cv::Mat( swapped.height(), swapped.width(), CV_8UC3, const_cast<uchar*>(swapped.bits()), swapped.bytesPerLine() ).clone();
       }
       // 8-bit, 1 channel
       case QImage::Format_Indexed8:
       {
          cv::Mat  mat( inImage.height(), inImage.width(), CV_8UC1, const_cast<uchar*>(inImage.bits()), inImage.bytesPerLine() );
          return (inCloneImageData ? mat.clone() : mat);
       }
       default:
          qWarning() << "ASM::QImageToCvMat() - QImage format not handled in switch:" << inImage.format();
          break;
    }
    return cv::Mat();
}

Matrix3f Utils::ransac2(QVector<Vector3f> pA, QVector<Vector3f> pB, double N, double threshold, bool adaptativeSearch, int randomSize)
{
    int pairsQuantity = std::min(pA.count(), pB.count());
    int ransacCounter = 0;
    QVector<int> inliers, maxInliers;
    Matrix3f Hfinal;
    double e = 0.5; // Outliers likelihood
    double p = 0.99;
    QVector<Vector3f> bestInliersA, bestInliersB;
    while(ransacCounter < N){
        QVector<int> randomPairsIndexes = selectRandomPairs(randomSize, pairsQuantity);

        QVector<Vector3f> randomPointsInFirstImage;
        QVector<Vector3f> randomPointsInSecondImage;
        for(int i =0 ; i < randomPairsIndexes.size(); i++ )
        {
            randomPointsInFirstImage.push_back(pA.at(randomPairsIndexes.at(i)));
            randomPointsInSecondImage.push_back(pB.at(randomPairsIndexes.at(i)));
        }
        Matrix3f Htemp;
        Htemp = calculate_H(randomPointsInFirstImage, randomPointsInSecondImage);

        inliers = getRansacInliers(pA, pB, Htemp, threshold);
        if(inliers.size() > maxInliers.size()){
            maxInliers = inliers;
            Hfinal = Htemp;
        }
        if(adaptativeSearch){
            e = 1 - ((float)inliers.size()/pairsQuantity);
            N = log(1-p)/log(1 - pow(1 - e, randomSize));
        }
        ransacCounter++;
        cout << N << endl;
    }
    qDebug() << ransacCounter;
    qDebug() << maxInliers.size();
    for(int i =0 ; i < maxInliers.size(); i++ )
    {
        bestInliersA.push_back(pA.at(maxInliers.at(i)));
        bestInliersB.push_back(pB.at(maxInliers.at(i)));
    }
    cout << "H do ransac2 antes de chamar gaussNewton" << endl;
    cout << Hfinal << endl;
    cout << endl;
    Matrix3f HfinalA = Hfinal;
    VectorXf A(Map<VectorXf>(Hfinal.data(), Hfinal.cols()*Hfinal.rows()));
    //cout << "Norma de H antes do gaussNewton" << endl;
    //cout << A.squaredNorm() << endl;
    Hfinal = gaussNewton(Hfinal, bestInliersA, bestInliersB);
    Matrix3f HfinalB = Hfinal;
    VectorXf B(Map<VectorXf>(Hfinal.data(), Hfinal.cols()*Hfinal.rows()));
    //cout << "Norma de H depois do gaussNewton" << endl;
    //cout << B.squaredNorm() << endl;

    if (A.squaredNorm() < B.squaredNorm())
    {
        cout << "Erro de +" << (( B.squaredNorm() - A.squaredNorm() ) / A.squaredNorm())*100 << "%" << endl;
    }
    else
    {
        cout << "Erro de -" << (( A.squaredNorm() - B.squaredNorm() ) / A.squaredNorm())*100 << "%" << endl;
    }

    if (HfinalA.norm() < HfinalB.norm())
    {
        cout << "Erro de +" << (( HfinalB.norm() - HfinalA.norm() ) / HfinalA.norm())*100 << "%" << endl;
    }
    else
    {
        cout << "Erro de -" << (( HfinalA.norm() - HfinalB.norm() ) / HfinalA.norm())*100 << "%" << endl;
    }

    return Hfinal;
}

QVector<int> Utils::selectRandomPairs(int numberOfCorrespondences, int size)
{
    QVector<int> selectedIndexes;
    for (int j = 0; j < numberOfCorrespondences; j++)
    {
        int correspondence = rand() % size;
        selectedIndexes.push_back(correspondence);
    }
    return selectedIndexes;
}

QVector<int> Utils::getRansacInliers(QVector<Vector3f> pA, QVector<Vector3f> pB, Matrix3f H, float threshold)
{
    QVector<int> inliers;
    for(int i = 0; i < pA.size(); i++){
        Vector3f v;
        v = H * pA.at(i);
        v/=v(2);
        float distance = squaredEuclideanDistance(v, pB.at(i));
        if(distance < threshold){
            inliers.push_back(i);
        }
    }
    return inliers;
}

MatrixXf Utils::calculate_H(QVector <Vector3f> bp, QVector <Vector3f> rp)
{
    //CREATE MATRIX
    MatrixXf m(8, 8);
    for(int i = 0; i< rp.count(); i++){
        MatrixXf lineA(1,8);
        lineA << bp[i](0), bp[i](1), 1, 0, 0, 0, -(bp[i](0)*rp[i](0)), -(bp[i](1)*rp[i](0));
        MatrixXf lineB(1,8);
        lineB << 0, 0, 0, bp[i](0), bp[i](1), 1, -(bp[i](0)*rp[i](1)), -(bp[i](1)*rp[i](1));
        m.row(i*2) << lineA;
        m.row(i*2 + 1) << lineB;
    }
    MatrixXf A  = m.inverse();
    MatrixXf B(8,1);
    for(int i = 0; i< rp.count(); i++){
        B.row(i*2) << rp.at(i)(0);
        B.row(i*2 +1) << rp.at(i)(1);
    }
    MatrixXf v = A*B;
    Matrix3f h;

    h << v(0), v(1), v(2), v(3), v(4), v(5), v(6), v(7), 1;

    return h;
}

float Utils::squaredEuclideanDistance(Vector3f a, Vector3f b)
{
    float distance = 0;
    for (int i = 0; i < a.size(); i++)
         distance += pow( (a(i) - b(i)), 2);
    return distance;
}

QVector< QVector<Vector3f> > Utils::sift2(const char *img1Path, const char * img2Path)
{
    QVector< QVector<Vector3f> > goodPairs;

    QImage inputImage1 = QImage(img1Path);
    QImage inputImage2 = QImage(img2Path);

    //Mat img_1 = imread( img1Path, CV_LOAD_IMAGE_GRAYSCALE );
    //Mat img_2 = imread( img2Path, CV_LOAD_IMAGE_GRAYSCALE );

    Mat img_1 = Utils::QImage2Mat(inputImage1);
    Mat img_2 = Utils::QImage2Mat(inputImage2);

    if( !img_1.data || !img_2.data )
        return goodPairs;

      //-- Step 1: Detect the keypoints using SIFT Detector

      SiftFeatureDetector detector;

      std::vector<KeyPoint> keypoints_1, keypoints_2;

      detector.detect( img_1, keypoints_1 );
      detector.detect( img_2, keypoints_2 );

      //-- Step 2: Calculate descriptors (feature vectors)
      SiftDescriptorExtractor extractor;

      Mat descriptors_1, descriptors_2;

      extractor.compute( img_1, keypoints_1, descriptors_1 );
      extractor.compute( img_2, keypoints_2, descriptors_2 );

      //-- Step 3: Matching descriptor vectors using FLANN matcher
      FlannBasedMatcher matcher;
      std::vector< DMatch > matches;
      matcher.match( descriptors_1, descriptors_2, matches );

      double max_dist = 0; double min_dist = 100;

      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < (int)matches.size(); i++ )
      {
        double dist = matches[1].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }

      //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
      //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
      //-- small)
      //-- PS.- radiusMatch can also be used here.
      std::vector< DMatch > good_matches;

      for( int i = 0; i < (int)matches.size(); i++ )
      {
          if( matches[i].distance <= max(2*min_dist, 0.02) )
            good_matches.push_back( matches[i]);
      }

      QVector<Vector3f> listaTemp_0;
      QVector<Vector3f> listaTemp_1;
      for(int i = 1 ; i < (int)good_matches.size(); i++){
          float x = keypoints_1[good_matches[i].queryIdx].pt.x;
          float y = keypoints_1[good_matches[i].queryIdx].pt.y;
          Vector3f temp;
          temp << x, y, 1;
          listaTemp_0.push_back(temp);

          x = keypoints_2[good_matches[i].trainIdx].pt.x;
          y = keypoints_2[good_matches[i].trainIdx].pt.y;
          Vector3f temp2;
          temp2 << x, y, 1;
          listaTemp_1.push_back(temp2);
      }
      goodPairs.push_back(listaTemp_0);
      goodPairs.push_back(listaTemp_1);
      return goodPairs;
}


Matrix3f Utils::dlt2(QVector<Vector3f> pointsFirstImage, QVector<Vector3f> pointsSecondImage)
{
    MatrixXf A(pointsFirstImage.size()*2, 9);

    for(std::vector<int>::size_type i = 0; i < (std::vector<int>::size_type)pointsFirstImage.size(); i++) {

        Vector3f firstPoint, secondPoint;
        firstPoint << pointsFirstImage.at(i);
        secondPoint << pointsSecondImage.at(i);
        A.row(i * 2) << 0, 0, 0, -firstPoint(0), -firstPoint(1), -1, secondPoint(1)*firstPoint(0), secondPoint(1)*firstPoint(1), secondPoint(1);
        A.row(i * 2 + 1) << firstPoint(0), firstPoint(1), 1, 0, 0, 0, -secondPoint(0)*firstPoint(0), -secondPoint(0)*firstPoint(1), -secondPoint(0);

    }

    JacobiSVD<MatrixXf> SVD(A, Eigen::ComputeThinV);
    VectorXf h = SVD.matrixV().col(SVD.matrixV().cols() - 1);
    Matrix3f H;
    H <<    h(0), h(1), h(2),
            h(3), h(4), h(5),
            h(6), h(7), h(8);
    return H;
}

Matrix3f Utils::getTMatrix2(QVector<Vector3f> points)
{
    float u_avg = 0;
    float v_avg = 0;
    int u_sum = 0;
    int v_sum = 0;

    for(std::vector<int>::size_type i = 0; i < (std::vector<int>::size_type)points.size(); i++) {
        Vector3f point = points.at(i);
        u_sum += point(0);
        v_sum += point(1);
    }

    u_avg = u_sum / (float)points.size();
    v_avg = v_sum / (float)points.size();


    float sum = 0;
    float s = 0;
    for(std::vector<int>::size_type i = 0; i < (std::vector<int>::size_type)points.size(); i++) {
        Vector3f point = points.at(i);
        sum += sqrt(pow(((float)point(0) - u_avg), 2) + pow(((float)point(1) - v_avg), 2));
    }
    s = (sqrt(2) * (float)points.size()) / sum;

    Matrix3f T;
    T << s, 0, -s*u_avg,
         0, s, -s*v_avg,
         0, 0, 1;
    return T;
}

Matrix3f Utils::dltNormalized2(QVector<Vector3f> pointsFirstImage, QVector<Vector3f> pointsSecondImage)
{

    Matrix3f T = getTMatrix2(pointsFirstImage);
    Matrix3f T2 = getTMatrix2(pointsSecondImage);

    QVector<Vector3f> l1, l2;
    Vector3f p1, p2;

    for(std::vector<int>::size_type i = 0; i < (std::vector<int>::size_type)pointsFirstImage.size(); i++) {

        p1 << pointsFirstImage.at(i)(0), pointsFirstImage.at(i)(1), 1;
        p2 << pointsSecondImage.at(i)(0), pointsSecondImage.at(i)(1), 1;

        Vector3f res = T * p1;
        l1.push_back(res);
        res = T2 * p2;
        l2.push_back(res);

    }

    Matrix3f Hn = dlt2(l1, l2);
    Matrix3f H = T.inverse().eval() * Hn * T2;

    return H;

}

Matrix3f Utils::gaussNewton(Matrix3f H, QVector<Vector3f> pointsFirstImage, QVector<Vector3f> pointsSecondImage)
{
    MatrixXf Ji(2, 9);
    MatrixXf J(pointsFirstImage.size()*2, 9);
    VectorXf fh(pointsFirstImage.size()*2);
    VectorXf X(pointsFirstImage.size()*2);
    VectorXf fhTemp(pointsFirstImage.size()*2);
    VectorXf XTemp(pointsFirstImage.size()*2);
    Matrix3f HTemp;
    int counter = 0;
    double lambda = 1;
    double squaredNorm, squaredNormTemp;
    squaredNorm = squaredNormTemp = 0;
    H(0,2) = H(0,2) - 10;
    H(1,2) = H(1,2) - 10;
    do
    {
        counter++;
        for (int i = 0; i < (int)pointsFirstImage.size(); i++)
        {
            Vector3f xi = pointsFirstImage.at(i);
            Vector3f xilinha = pointsSecondImage.at(i);
            Ji << xi(0)/xilinha(2), xi(1)/xilinha(2), xi(2)/xilinha(2), 0, 0, 0, -xi(0)*xilinha(0)/xilinha(2), xi(1)*xilinha(0)/xilinha(2), xi(2)*xilinha(0)/xilinha(2),
                  0, 0, 0, xi(0)/xilinha(2), xi(1)/xilinha(2), xi(2)/xilinha(2), -xi(0)*xilinha(1)/xilinha(2), xi(1)*xilinha(1)/xilinha(2), xi(2)*xilinha(1)/xilinha(2);
            J.row(i*2) << Ji.row(0);
            J.row(i*2 + 1) << Ji.row(1);

            Vector3f Hxi = H*xi;
            Hxi /= Hxi(2);
            fh(i*2) = Hxi(0);
            fh(i*2 + 1) = Hxi(1);
            X(i*2) = xilinha(0);
            X(i*2 + 1) = xilinha(1);
         }

        MatrixXf JT(9, pointsFirstImage.size()*2);
        JT = J.transpose().eval();

        VectorXf deltaX(9);
        deltaX = (JT*J).inverse().eval()*(-JT)*fh;

        cout << "Erro mais externo: " << fabs(squaredNormTemp - squaredNorm) << endl;

        if ( counter > 1 && fabs(squaredNormTemp - squaredNorm) < 1e-8 )
            break;

        VectorXf diff = X - fh;
        squaredNorm = diff.squaredNorm();

        while(true)
        {
            HTemp(0,0) = H(0,0) + lambda*deltaX(0);
            HTemp(0,1) = H(0,1) + lambda*deltaX(1);
            HTemp(0,2) = H(0,2) + lambda*deltaX(2);
            HTemp(1,0) = H(1,0) + lambda*deltaX(3);
            HTemp(1,1) = H(1,1) + lambda*deltaX(4);
            HTemp(1,2) = H(1,2) + lambda*deltaX(5);
            HTemp(2,0) = H(2,0) + lambda*deltaX(6);
            HTemp(2,1) = H(2,1) + lambda*deltaX(7);
            HTemp(2,2) = H(2,2) + lambda*deltaX(8);

            for (int i = 0; i < (int)pointsFirstImage.size(); i++)
            {
                Vector3f xi = pointsFirstImage.at(i);
                Vector3f xilinha = pointsSecondImage.at(i);
                Vector3f Hxi = HTemp*xi;
                Hxi /= Hxi(2);
                fhTemp(i*2) = Hxi(0);
                fhTemp(i*2 + 1) = Hxi(1);
                XTemp(i*2) = xilinha(0);
                XTemp(i*2 + 1) = xilinha(1);
             }

             VectorXf diffTemp = XTemp - fhTemp;
             squaredNormTemp = diffTemp.squaredNorm();
             /*cout << endl;
             cout << "counter" << endl;
             cout << counter << endl;
             cout << "lambda" << endl;
             cout << lambda << endl;
             cout << "squaredNormTemp" << endl;
             cout << squaredNormTemp << endl;
             cout << "squaredNorm" << endl;
             cout << squaredNorm << endl;
             cout << endl;*/
             if (squaredNormTemp <= squaredNorm)
                 break;
             lambda /= 2;
        }

        H = HTemp;
        lambda = std::max(2*lambda, 1.0);
     } while (counter <= 1000);
     return H;
}

Matrix3f Utils::calculate_F(QVector<Vector3f> pA, QVector<Vector3f> pB)
{
    int pairsQuantity = std::min(pA.count(), pB.count());
    QVector<int> randomPairsIndexes = selectRandomPairs(8, pairsQuantity);

    QVector<Vector3f> l1, l2;
    for (int i=0; i < randomPairsIndexes.length(); i++)
    {
        l1.push_back(pA.at(randomPairsIndexes.at(i)));
        l2.push_back(pB.at(randomPairsIndexes.at(i)));
    }

    Matrix3f T1 = getTMatrix2(l1);
    Matrix3f T2 = getTMatrix2(l2);

    Matrix3f FTemp, FRestricted;
    MatrixXf A;

    for (int i = 0; i < 8; i++)
    {
        Vector3f first = l1.at(i);
        Vector3f second = l2.at(i);

        float x = (T1*first)(0);
        float y = (T1*first)(1);
        float xlinha = (T2*second)(0);
        float ylinha = (T2*second)(1);

        A.row(i) << xlinha*x, xlinha*y, xlinha, ylinha*x, ylinha*y, ylinha, x, y, 1;
    }

    JacobiSVD<MatrixXf> SVD(A, ComputeFullV);
    VectorXf x = SVD.matrixV().col(SVD.matrixV().cols() - 1);
    FTemp << x(0), x(1), x(2),
             x(3), x(4), x(5),
             x(6), x(7), x(8);

    JacobiSVD<MatrixXf> SVD2(FTemp, ComputeFullV | ComputeFullU);
    VectorXf singularValues;
    singularValues = SVD2.singularValues();
    DiagonalMatrix<float, 3,3> D(singularValues(0), singularValues(1), 0);
    Matrix3f DMat = D.toDenseMatrix();
    FRestricted = SVD2.matrixU() * DMat * SVD2.matrixV().transpose();

    return T2.transpose().eval()*FRestricted*T1;
}

Matrix3f Utils::build_K(float focalmm, float pixelSize_x, float pixelSize_y, float centerPx_x, float centerPx_y)
{
    Matrix3f K;

    float ax = focalmm * pixelSize_x;
    float ay = focalmm * pixelSize_y;

    float x0 = pixelSize_x * centerPx_x;
    float y0 = pixelSize_y * centerPx_y;

    K << ax, 0, x0,
         0, ay, y0,
         0,  0,  1;

    return K;
}

Matrix3f Utils::calculate_E(Matrix3f F, Matrix3f K, Matrix3f Kl)
{
    Matrix3f E;

    E = Kl.transpose().eval() * F * K;

    Eigen::JacobiSVD<Eigen::MatrixXf> SVD(E, Eigen::ComputeFullV | Eigen::ComputeFullU);

    Eigen::DiagonalMatrix<float, 3,3> D(1,1, 0);
    Eigen::Matrix3f DMat = D.toDenseMatrix();
    E = SVD.matrixU() * DMat * SVD.matrixV().transpose();

    return E;
}

MatrixXf Utils::calculate_P(Matrix3f E, Vector3f pA, Vector3f pB)
{
    Eigen::Matrix3f W;
    W <<   0,  -1,  0,
           1,   0,  0,
           0,   0,  1;
    Eigen::Matrix3f Z;
    Z <<   0,  1,  0,
          -1,  0,  0,
           0,  0,  0;
    Eigen::Matrix3f U, V;
    Eigen::JacobiSVD<Eigen::MatrixXf> SVD(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = SVD.matrixU();
    V = SVD.matrixV();

   if((U*V.transpose()).determinant() < 0)
   {
       V.col(2) *= -1;
   }

   Eigen::Vector3f u3;
   u3 = U.col(2);
   Eigen::MatrixXf P1(3,4);
   Eigen::MatrixXf P2(3,4);
   Eigen::MatrixXf P3(3,4);
   Eigen::MatrixXf P4(3,4);

   P1.topLeftCorner(3,3) = U * W * V.transpose();
   P1.col(3) = u3;

   P2.topLeftCorner(3,3) = U * W * V.transpose();
   P2.col(3) = -u3;

   P3.topLeftCorner(3,3) = U * W.transpose() * V.transpose();
   P3.col(3) = u3;

   P4.topLeftCorner(3,3) = U * W.transpose() * V.transpose();
   P4.col(3) = -u3;

   Vector3f X1, X2;
   MatrixXf P;

   P << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0;

   X1 = P.inverse().eval() * pA;
   X2 = P1.inverse().eval() * pB;

   if (X1(2) >= 0 && X2(2) >= 0)
       return P1;

   X1 = P.inverse().eval() * pA;
   X2 = P2.inverse().eval() * pB;

   if (X1(2) >= 0 && X2(2) >= 0)
       return P2;

   X1 = P.inverse().eval() * pA;
   X2 = P3.inverse().eval() * pB;

   if (X1(2) >= 0 && X2(2) >= 0)
       return P3;

   X1 = P.inverse().eval() * pA;
   X2 = P4.inverse().eval() * pB;

   if (X1(2) >= 0 && X2(2) >= 0)
       return P4;

}

QVector<VectorXf> Utils::get3DPointsByTriangulation(QVector<Vector3f> pA, QVector<Vector3f> pB, MatrixXf P, MatrixXf Pl)
{
    QVector<VectorXf> points;
    Matrix4f A;
    for (int i=0; i < pA.size(); i++)
    {
        Vector3f a = pA.at(i);
        Vector3f b = pB.at(i);

        Eigen::Vector4f v0, v1, v2, v3;
        v0 = a(0) * P.row(2) - P.row(0);
        v1 = a(1) * P.row(2) - P.row(1);
        v2 = b(0) * Pl.row(2) - Pl.row(0);
        v3 = b(1) * Pl.row(2) - Pl.row(1);

        A << v0.transpose(), v1.transpose(), v2.transpose(), v3.transpose();
        Eigen::JacobiSVD<Eigen::MatrixXf> SVD(A, Eigen::ComputeFullV);
        Eigen::VectorXf X = SVD.matrixV().col(SVD.matrixV().cols() - 1);
        X /= X(3);

        points.push_back(X);
    }
    return points;

}
