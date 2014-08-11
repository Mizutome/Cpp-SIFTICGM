TEMPLATE = app
CONFIG += console
CONFIG -= qt
CONFIG += link_pkgconfig

PKGCONFIG += opencv
PKGCONFIG += gtk+-2.0

SOURCES += main.cpp \
    CGVM.cpp \
    imgfeatures.cpp \
    kdtree.cpp \
    minpq.cpp \
    sift.cpp \
    utils.cpp \
    xform.cpp \
    ipoint.cpp

HEADERS += CGVM.h \
    imgfeatures.h \
    kdtree.h \
    minpq.h \
    sift.h \
    utils.h \
    xform.h \
    ipoint.h \
    surflib.h

QMAKE_CXXFLAGS += -fpermissive
