TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


win32{
INCLUDEPATH += C:\opencv343\build\include ### was c:/opencv343/release/install/include
LIBS += -LC:\openCV343\build\x64\vc15\lib
LIBS +=    -lopencv_world343 \
    -lws2_32 \
##    -lopencv_ffmpeg343
}

unix {
## MACOSX & LINUX
INCLUDEPATH += "/usr/local//include/opencv4"  #MACOS X brew installed opencv4
INCLUDEPATH += "/usr/local//include/"  #linux location
LIBS += -L/usr/local/lib
LIBS += -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_imgcodecs \
    -lopencv_features2d \
    -lopencv_calib3d \
    -lopencv_videoio \
    -lopencv_objdetect
#    -lopencv_ffmpeg \
##    -lws2_32 \
}

SOURCES += \
    ../../Sources/Opencv_disparity/stereo_match.cpp
