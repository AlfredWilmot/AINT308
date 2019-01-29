TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


INCLUDEPATH += C:\opencv343\build\include ### was c:/opencv343/release/install/include

#LIBS += -LC:\opencv343\build\install\x86\mingw\lib
#LIBS += -LC:\opencv343\build\install\x86\mingw\bin
LIBS += -LC:\openCV343\build\x64\vc15\lib
LIBS +=    -lopencv_world343d \
    -lws2_32 \
##    -lopencv_ffmpeg343

SOURCES += \
    ../../Sources/Owl-1/main.cpp

HEADERS += \
    ../../Sources/Owl-1/owl-comms.h \
    ../../Sources/Owl-1/owl-cv.h \
    ../../Sources/Owl-1/owl-pwm.h
