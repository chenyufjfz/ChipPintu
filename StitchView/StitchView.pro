#-------------------------------------------------
#
# Project created by QtCreator 2016-06-26T10:21:51
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = StitchView
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    stitchview.cpp

HEADERS  += mainwindow.h \
    stitchview.h

FORMS    += mainwindow.ui

#DESTDIR = $$_PRO_FILE_PWD_/../app


Release:LIBS += -L$$_PRO_FILE_PWD_/../lib/release
Release:LIBS += -lopencv_calib3d249 -lopencv_contrib249 -lopencv_core249 -lopencv_features2d249 -lopencv_flann249
Release:LIBS += -lopencv_gpu249 -lopencv_highgui249 -lopencv_imgproc249 -lopencv_legacy249 -lopencv_ml249
Release:LIBS += -lopencv_nonfree249 -lopencv_objdetect249 -lopencv_photo249 -lopencv_stitching249 -lopencv_superres249
Release:LIBS += -lopencv_ts249 -lopencv_video249 -lopencv_videostab249
Release:INCLUDEPATH += $$_PRO_FILE_PWD_/../cvinclude/release/

Debug:LIBS += -L$$_PRO_FILE_PWD_/../lib/debug
Debug:LIBS += -lopencv_calib3d249d -lopencv_contrib249d -lopencv_core249d -lopencv_features2d249d -lopencv_flann249d
Debug:LIBS += -lopencv_gpu249d -lopencv_highgui249d -lopencv_imgproc249d -lopencv_legacy249d -lopencv_ml249d
Debug:LIBS += -lopencv_nonfree249d -lopencv_objdetect249d -lopencv_photo249d -lopencv_stitching249d -lopencv_superres249d
Debug:LIBS += -lopencv_ts249d -lopencv_video249d -lopencv_videostab249d
Debug:INCLUDEPATH += $$_PRO_FILE_PWD_/../cvinclude/debug/
