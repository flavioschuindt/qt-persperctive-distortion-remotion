#ifndef LINE_H
#define LINE_H

#include <QLabel>
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>

#include "dot.h"

class Line: public QLabel
{
    Q_OBJECT
public:
    explicit Line(QWidget *parent = 0);
    ~Line();

    Dot *getA() const;
    void setA(Dot *value);

    Dot *getB() const;
    void setB(Dot *value);

protected:
    void paintEvent(QPaintEvent *e);
private:
        Dot *a;
        Dot *b;
};

#endif // LINE_H
