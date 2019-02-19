QT += core
QT -= gui

TARGET = NetAnalysis
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app
DESTDIR = $$_PRO_FILE_PWD_/../app

SOURCES += main.cpp \
        lex.yy.cpp \
        spice.tab.cpp

HEADERS  += spice.tab.hpp

