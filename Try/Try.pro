QT += core

TARGET = Try
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    $$_PRO_FILE_PWD_/../ViaWireExtract/iclayer.cpp

HEADERS += $$_PRO_FILE_PWD_/../ViaWireExtract/iclayer.h

DESTDIR = $$_PRO_FILE_PWD_/../app

Debug:LIBS += -L$$_PRO_FILE_PWD_/../lib/debug
Release:LIBS += -L$$_PRO_FILE_PWD_/../lib/release

LIBS += -lMdb
INCLUDEPATH += $$_PRO_FILE_PWD_/../Mdb $$_PRO_FILE_PWD_/../ViaWireExtract
win32:LIBS += -ladvapi32
unix:LIBS += -lpthread
