#-------------------------------------------------
#
# Project created by QtCreator 2015-06-21T18:12:04
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MyActions
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    picture.cpp \
    dot.cpp \
    utils.cpp \
    line.cpp \
    auxiliarwindow.cpp

HEADERS  += mainwindow.h \
    picture.h \
    dot.h \
    utils.h \
    line.h \
    auxiliarwindow.h

FORMS    += mainwindow.ui \
    auxiliarwindow.ui

RESOURCES += \
    myres.qrc \
    myres2.qrc

INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/local/include/opencv2

LIBS += -lopencv_core
LIBS += -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_features2d
LIBS += -lopencv_nonfree
LIBS += -lopencv_flann
