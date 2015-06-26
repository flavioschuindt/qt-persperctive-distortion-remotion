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
           Dot* dot = new Dot(this);
           dot->show();
           update();
           dot->move(e->x(), e->y());
    }

}

