#include "line.h"

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
    painter.drawLine(a->x(), a->y(), b->x(), b->y());
    painter.end();
}
Dot *Line::getB() const
{
    return b;
}

void Line::setB(Dot *value)
{
    b = value;
}

Dot *Line::getA() const
{
    return a;
}

void Line::setA(Dot *value)
{
    a = value;
}

