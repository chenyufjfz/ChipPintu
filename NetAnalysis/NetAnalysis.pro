QT += core concurrent
QT -= gui

TARGET = NetAnalysis
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app
DESTDIR = $$_PRO_FILE_PWD_/../app

SOURCES += main.cpp \
        lex.yy.cpp \
        spice.tab.cpp \
    circuit.cpp \
    circuitmatch.cpp

HEADERS  += spice.tab.hpp \
    circuit.h \
    circuitmatch.h

win32 {
LIBS += -lDbghelp
LIBS += -ladvapi32
}
