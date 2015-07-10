#ifndef DOT_H
#define DOT_H

#include <QLabel>
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>

class Dot: public QLabel
{
    Q_OBJECT
public:
    explicit Dot(QWidget *parent = 0);
    ~Dot();
    protected:
    void paintEvent(QPaintEvent *e);
};

#endif // DOT_H
