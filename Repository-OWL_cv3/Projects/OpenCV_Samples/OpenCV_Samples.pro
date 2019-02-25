TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


INCLUDEPATH += C:\OpenCV31\build\include ### was c:/opencv31/release/install/include

#LIBS += -LC:\opencv31\build\install\x86\mingw\lib
#LIBS += -LC:\opencv31\build\install\x86\mingw\bin
#LIBS += -LC:\OpenCV31\build\install\x64\mingw\lib
LIBS += -LC:\opencv31\build\lib
LIBS += -lopencv_core310 \
    -lopencv_highgui310 \
    -lopencv_imgproc310 \
    -lopencv_features2d310 \
    -lopencv_calib3d310 \
    -lopencv_videoio310 \
    -lws2_32 \
##    -lopencv_ffmpeg310

SOURCES += \
    ../../Sources/OpenCV_samples/contours2.cpp
