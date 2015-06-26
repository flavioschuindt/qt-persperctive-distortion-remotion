#ifndef PICTURE_H
#define PICTURE_H

#include <QLabel>

class Picture: public QLabel
{
    Q_OBJECT
public:
    Picture(QWidget *parent = 0);
    ~Picture();
protected:
    void mousePressEvent(QMouseEvent *e);
};

#endif // PICTURE_H
