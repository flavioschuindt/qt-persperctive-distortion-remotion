#include "dot.h"
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>

Dot::Dot(QWidget *parent)
    : QLabel(parent)
{
    resize(10, 10);
}

Dot::~Dot()
{

}

void Dot::paintEvent(QPaintEvent *e)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setBrush(Qt::red);
    QPen paintpen(Qt::red);
    paintpen.setWidth(9);
    QPoint p1;
    p1.setX(0);
    p1.setY(0);
    painter.setPen(paintpen);
    painter.drawPoint(p1);
}

