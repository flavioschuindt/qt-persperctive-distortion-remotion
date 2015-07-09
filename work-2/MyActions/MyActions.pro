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
    line.cpp

HEADERS  += mainwindow.h \
    picture.h \
    dot.h \
    utils.h \
    line.h

FORMS    += mainwindow.ui

RESOURCES += \
    myres.qrc

INCLUDEPATH += /usr/include/eigen3
