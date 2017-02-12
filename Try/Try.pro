QT += core

TARGET = Try
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    iclayer.cpp

HEADERS += \
    iclayer.h

DESTDIR = $$_PRO_FILE_PWD_/../app

Debug:LIBS += -L$$_PRO_FILE_PWD_/../lib/debug
Release:LIBS += -L$$_PRO_FILE_PWD_/../lib/release

LIBS += -lMdb
INCLUDEPATH += $$_PRO_FILE_PWD_/../Mdb
win32:LIBS += -ladvapi32
unix:LIBS += -lpthread
