#include "picture.h"
#include "dot.h"
#include "utils.h"
#include "line.h"

QList<Dot*> Picture::selectedPoints;

Picture::Picture(QWidget *parent)
    : QLabel(parent)
{
}

Picture::~Picture()
{

}

void Picture::mousePressEvent(QMouseEvent *e)
{

}

