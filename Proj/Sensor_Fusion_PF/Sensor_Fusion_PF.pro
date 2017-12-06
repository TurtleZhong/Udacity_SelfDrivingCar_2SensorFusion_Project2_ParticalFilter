QT += core
QT -= gui

CONFIG += c++11

TARGET = Sensor_Fusion_PF
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += \
    ../../src/main.cpp \
    ../../src/particle_filter.cpp

HEADERS += \
    ../../src/helper_functions.h \
    ../../src/json.hpp \
    ../../src/map.h \
    ../../src/particle_filter.h
