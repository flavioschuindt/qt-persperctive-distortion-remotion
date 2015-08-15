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
    if (METHOD == 0) // Method to click on points on both images
    {
        if (e->button() == Qt::LeftButton) {
            Dot* dot = new Dot(this);
            dot->show();
            update();
            dot->move(e->x(), e->y());
            selectedPoints.push_back(dot);
        }
    }

}

