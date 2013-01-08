######################################################################
# Project File for EfPiSoft
#
# Created out of template.
# If you correctly installed QT, just type "qmake efpisoft_linux.pro"
# to create the makefile.
#
######################################################################

TEMPLATE = app

INCLUDEPATH += .
INCLUDEPATH += ./coin_include
INCLUDEPATH += ./include

LIBS += -lXi
LIBS += -L/usr/local/lib -lSoQt
LIBS += -L./lib -lCoin -l jmesh

# Input
HEADERS += efpisoft.h fittingPrimitives.h
SOURCES += efpisoft.cpp fittingPrimitives.cpp
