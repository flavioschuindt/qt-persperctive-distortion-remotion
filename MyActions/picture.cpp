#include "picture.h"
#include "dot.h"
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>

Picture::Picture(QWidget *parent)
    : QLabel(parent)
{

}

Picture::~Picture()
{

}

void Picture::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton) {

           QMessageBox::information(this, "title", QString::number(e->x()) + " - " + QString::number(e->y()));
           //QPainter painter(new Dot(this));
           /*QImage tmp(this->pixmap()->toImage());
           QPainter painter(&tmp);
           QPen paintpen(Qt::red);
           paintpen.setWidth(7);
           QPoint p1;
           p1.setX(e->x());
           p1.setY(e->y());
           painter.setPen(paintpen);
           painter.drawPoint(p1);
           this->setPixmap(QPixmap::fromImage(tmp));*/
           Dot* dot = new Dot(this);
           dot->move(e->x(), e->y());
           update();
    }

}

