#-------------------------------------------------
#
# Project created by QtCreator 2017-02-12T21:40:58
#
#-------------------------------------------------

QT       += core gui

TARGET = ICLayerLib
TEMPLATE = lib
CONFIG += staticlib


CONFIG(debug, debug|release) {
TARGET = ICLayerd
LIBS += -L$$_PRO_FILE_PWD_/../lib/debug
DESTDIR = $$_PRO_FILE_PWD_/../../ChipAnalysis/lib/debug
} else {
TARGET = ICLayer
LIBS += -L$$_PRO_FILE_PWD_/../lib/release
DESTDIR = $$_PRO_FILE_PWD_/../../ChipAnalysis/lib/release
}

LIBS += -lMdb
INCLUDEPATH += $$_PRO_FILE_PWD_/../Mdb

Release:LIBS += -L$$_PRO_FILE_PWD_/../lib/release
Release:LIBS += -lopencv_core249 -lopencv_highgui249 -lopencv_imgproc249
Release:INCLUDEPATH += $$_PRO_FILE_PWD_/../cvinclude/release/

Debug:LIBS += -L$$_PRO_FILE_PWD_/../lib/debug
Debug:LIBS += -lopencv_core249d -lopencv_highgui249d -lopencv_imgproc249d
Debug:INCLUDEPATH += $$_PRO_FILE_PWD_/../cvinclude/debug/

SOURCES += $$_PRO_FILE_PWD_/../ViaWireExtract/iclayer.cpp

HEADERS += $$_PRO_FILE_PWD_/../ViaWireExtract/iclayer.h

unix {
    target.path = /usr/lib
    INSTALLS += target
    LIBS += -lpthread
}

win32:LIBS += -ladvapi32
