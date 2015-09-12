#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QMessageBox>
#include <QLabel>
#include <Qt>
#include "ui_mainwindow.h"
#include "utils.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void createInitialMatrixes();

private slots:
    void on_actionOpen_triggered();

    void on_actionSave_triggered();

private:
    Ui::MainWindow *ui;
    Matrix3f K;
    Matrix3f Kl;
    MatrixXf P;
    MatrixXf Pl;
    Matrix3f R1;
    Matrix3f R2;
    MatrixXf Rt1;
    MatrixXf Rt2;
    Vector3f t1;
    Vector3f t2;
    std::vector<Eigen::Vector3f> Points3D;
};

#endif // MAINWINDOW_H
