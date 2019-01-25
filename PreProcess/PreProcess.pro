QT += core
QT -= gui

TARGET = PreProcess
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

DESTDIR = $$_PRO_FILE_PWD_/../app

Release:LIBS += -L$$_PRO_FILE_PWD_/../lib/release
Release:LIBS += -lopencv_core249 -lopencv_highgui249 -lopencv_imgproc249
Release:INCLUDEPATH += $$_PRO_FILE_PWD_/../cvinclude/release/

Debug:LIBS += -L$$_PRO_FILE_PWD_/../lib/debug
Debug:LIBS += -lopencv_core249d -lopencv_highgui249d -lopencv_imgproc249d
Debug:INCLUDEPATH += $$_PRO_FILE_PWD_/../cvinclude/debug/

win32:LIBS += -ladvapi32
unix:LIBS += -lpthread
