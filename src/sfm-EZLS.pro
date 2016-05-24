#-------------------------------------------------
#
# Project created by QtCreator 2016-04-06T20:01:34
#
#-------------------------------------------------

QT       += core gui opengl xml widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = sfm-EZLS
TEMPLATE = app

#QTPLUGIN     += qjpeg

SOURCES +=  main.cpp               \
            mainwindow.cpp         \
            matching.cpp           \
            findcameramatrices.cpp \
            common.cpp             \
            Triangulation.cpp \
            sfmviewer.cpp

HEADERS  += mainwindow.h           \
            matching.h             \
            findcameramatrices.h   \
            common.h               \
            Triangulation.h \
            sfmviewer.h

FORMS    += mainwindow.ui

INCLUDEPATH += "/usr/local/include/"
CONFIG   += link_pkgconfig
PKGCONFIG+= opencv



INCLUDEPATH +=  "/usr/include/pcl-1.7/"    \
                "/usr/include/pcl-1.7/pcl" \
                "/usr/include/flann/"      \
                "/usr/include/eigen3/"     \



LIBS += -lpcl_common            \
        -lpcl_features          \
        -lpcl_filters           \
        -lpcl_io                \
        -lpcl_io_ply            \
        -lpcl_kdtree            \
        -lpcl_keypoints         \
        -lpcl_octree            \
        -lpcl_outofcore         \
        -lpcl_features          \
        -lpcl_visualization     \
        -lflann                 \
        -lqhull                 \
        -lboost_system          \


INCLUDEPATH += /home/sy/lib/libQGLViewer-2.6.3
LIBS += -L/home/sy/lib/libQGLViewer-2.6.3/QGLViewer
LIBS += -lQGLViewer
LIBS += -lglut
