#ifndef LINE_H
#define LINE_H

#include <QLabel>

class Line: public QLabel
{
    Q_OBJECT
public:
    explicit Line(QWidget *parent = 0);
    Line(int x1, int y1, int x2, int y2);
    ~Line();
    int getX1() const;
    void setX1(int value);

    int getX2() const;
    void setX2(int value);

    int getY1() const;
    void setY1(int value);

    int getY2() const;
    void setY2(int value);

protected:
    void paintEvent(QPaintEvent *e);
private:
        int x1;
        int x2;
        int y1;
        int y2;
};

#endif // LINE_H
