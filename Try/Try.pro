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
