TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
CONFIG -= core gui

SOURCES += main.cpp



INCLUDEPATH += c:/opencv3.1/release/install/include

LIBS += -LC:\opencv3.1\release\install\x86\mingw\lib
LIBS += -LC:\opencv3.1\release\install\x86\mingw\bin
LIBS +=    -lopencv_core310 \
    -lopencv_highgui310 \
    -lopencv_imgproc310 \
    -lopencv_features2d310 \
    -lopencv_calib3d310 \
    -lopencv_videoio310 \
    -lws2_32 \
##    -lopencv_ffmpeg310

HEADERS += \
    owl-pwm.h \
    owl-comms.h \
    owl-cv.h
