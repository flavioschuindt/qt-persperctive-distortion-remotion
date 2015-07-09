#include "line.h"
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>

Line::Line(QWidget *parent)
    : QLabel(parent)
{
    resize(1024, 1024);
}

Line::~Line()
{

}

void Line::paintEvent(QPaintEvent *e)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(Qt::red, 2));
    painter.drawLine(x1, y1, x2, y2);
    painter.end();
}
int Line::getY2() const
{
    return y2;
}

void Line::setY2(int value)
{
    y2 = value;
}

int Line::getY1() const
{
    return y1;
}

void Line::setY1(int value)
{
    y1 = value;
}

int Line::getX2() const
{
    return x2;
}

void Line::setX2(int value)
{
    x2 = value;
}

int Line::getX1() const
{
    return x1;
}

void Line::setX1(int value)
{
    x1 = value;
}


