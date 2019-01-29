TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


INCLUDEPATH += "C:/openCV343/build/include" ## CHECK LOCATION
LIBS += -LC:\opencv343\build\x64\vc15\lib ## CHECK LOCATION
LIBS += -LC:\opencv343\build\x64\vc15\bin
LIBS += -LC:\opencv343\build\lib
LIBS += -lopencv_world343.dll \  # assume opencv world is available, else have to include all libraries, as for unix, but specific ref to version
    -lws2_32


SOURCES += \
    ../../Sources/OpenCV_samples/contours2.cpp
