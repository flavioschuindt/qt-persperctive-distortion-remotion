#include "mainwindow.h"
#include "auxiliarwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    //AuxiliarWindow aux;
    //aux.show();

    return a.exec();
}
