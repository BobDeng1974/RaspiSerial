TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    raspiserial.cpp

HEADERS += \
    raspiserial.h \
    wiringSerial.h \
    wiringPi.h
