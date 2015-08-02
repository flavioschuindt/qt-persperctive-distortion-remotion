#ifndef AUXILIARWINDOW_H
#define AUXILIARWINDOW_H

#include <QMainWindow>

namespace Ui {
class AuxiliarWindow;
}

class AuxiliarWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit AuxiliarWindow(QWidget *parent = 0);
    ~AuxiliarWindow();

private slots:
    void on_actionOpen_triggered();

private:
    Ui::AuxiliarWindow *ui;
};

#endif // AUXILIARWINDOW_H
