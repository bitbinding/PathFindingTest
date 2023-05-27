#-------------------------------------------------
#
# Project created by QtCreator 2016-03-09T10:04:04
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PathFindingTest
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    pathfindingwidget.cpp \
    tracedialog.cpp

HEADERS  += mainwindow.h \
    pathfindingwidget.h \
    tracedialog.h

FORMS    += mainwindow.ui \
    tracedialog.ui

RC_FILE += PathFindingTest.rc

DISTFILES += \
    PathFindingTest.rc

