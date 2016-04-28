#-------------------------------------------------
#
# Project created by QtCreator 2016-04-06T20:01:34
#
#-------------------------------------------------

QT       += core gui

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
    visualization.cpp

HEADERS  += mainwindow.h           \
            matching.h             \
            findcameramatrices.h   \
            common.h               \
            Triangulation.h \
    visualization.h

FORMS    += mainwindow.ui

INCLUDEPATH += "/usr/local/include/"
CONFIG   += link_pkgconfig
PKGCONFIG+= opencv



INCLUDEPATH +=  "/usr/include/pcl-1.7/"    \
                "/usr/include/pcl-1.7/pcl" \
                "/usr/include/flann/"      \
                "/usr/include/eigen3/"     \
                "/usr/include/vtk-5.8"     \
                "/usr/include/boost"


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
#        -lpcl_segmentation      \
#        -lpcl_people            \
#        -lpcl_recognition       \
#        -lpcl_registration      \
#        -lpcl_sample_consensus  \
#        -lpcl_search            \
#        -lpcl_surface           \
#        -lpcl_tracking          \
        -lpcl_visualization     \
        -lflann      \
        -lqhull     \
        -lboost_system          \
        -lQVTK \
        -lvtkCommon \
        -lvtkFiltering \
        -lvtkRendering \
