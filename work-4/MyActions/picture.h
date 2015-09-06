#ifndef PICTURE_H
#define PICTURE_H

#include "line.h"
#include <QLabel>
#include <Eigen/Dense>
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>
#include <QPicture>


using namespace Eigen;
using namespace std;


class Picture: public QLabel
{
    Q_OBJECT
public:
    Picture(QWidget *parent = 0);
    ~Picture();
protected:
    void mousePressEvent(QMouseEvent *e);
    bool isReadyToProcessUserInputedData(QMouseEvent *e, int totalLines);
public:
    static QList<Dot*> selectedPoints;
};

#endif // PICTURE_H
