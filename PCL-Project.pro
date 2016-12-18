#-------------------------------------------------
#
# Project created by QtCreator 2016-11-29T11:55:12
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PCL-Project
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    Libs/pointcloudvisualizer.cpp \
    Libs/pointcloud.cpp \
    Libs/cropbox.cpp \
    Libs/threadmanager.cpp \
    Libs/taskmanager.cpp \
    controlbar.cpp

HEADERS  += mainwindow.h \
    Libs/pointcloudvisualizer.h \
    Libs/pointcloud.h \
    Libs/cropbox.h \
    Libs/threadmanager.h \
    Libs/taskmanager.h \
    controlbar.h

FORMS    += mainwindow.ui \
    controlbar.ui

DISTFILES += \
    CMakeLists.txt
