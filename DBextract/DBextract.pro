QT += core concurrent

TARGET = DBextract
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    $$_PRO_FILE_PWD_/../ViaWireExtract/iclayer.cpp


HEADERS += $$_PRO_FILE_PWD_/../ViaWireExtract/iclayer.h

DESTDIR = $$_PRO_FILE_PWD_/../app

Release:LIBS += -L$$_PRO_FILE_PWD_/../lib/release
Release:LIBS += -lopencv_core249 -lopencv_highgui249 -lopencv_imgproc249
Release:INCLUDEPATH += $$_PRO_FILE_PWD_/../cvinclude/release/

Debug:LIBS += -L$$_PRO_FILE_PWD_/../lib/debug
Debug:LIBS += -lopencv_core249d -lopencv_highgui249d -lopencv_imgproc249d
Debug:INCLUDEPATH += $$_PRO_FILE_PWD_/../cvinclude/debug/

LIBS += -lMdb
INCLUDEPATH += $$_PRO_FILE_PWD_/../Mdb $$_PRO_FILE_PWD_/../ViaWireExtract
win32:LIBS += -ladvapi32
unix:LIBS += -lpthread
