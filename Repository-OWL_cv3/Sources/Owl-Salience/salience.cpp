// owl.cpp : Defines the entry point for the console application.
/* Phil Culverhouse, James Rogers Oct 2016 (c) Plymouth University
 *
 * Uses IP sockets to communicate to the owl robot (see owl-comms.h)
 * Uses OpenCV to perform normalised cross correlation to find a match to a template
 * (see owl-cv.h).
 * PWM definitions for the owl servos are held in owl-pwm.h
 * includes bounds check definitions
 * requires setting for specific robot
 *
 * This demonstration programs does the following:
 * Implements a version of Itti & Kochs saliency model of saccadic stereo vision
 * 1. camera calibration is read from previous calibration data
 * 2. Main Loop - get stereo pair from camera stream
 * 3.   - correct for distorations using REMAP()
 * 4.   -
 *
 * First start communcations on Pi by running './OWLsocket'
 * Then run this program. The Pi server prints out [Rx Ry Lx Ly] pwm values and loops
 *
 *
 */

#define PI 3.14159265

#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

#include "user_made_code.h"

#include "opencv2/calib3d.hpp"



#include <iostream> // for standard I/O
#include <string>   // for strings


#define PX2DEG  0.0768
#define DEG2PWM 10.730
#define IPD 58.3

#define xOffset  -0.3815
#define yOffset -54.6682
#define xScale   26.4842
#define yScale   91.4178

using namespace std;
using namespace cv;

// PFC/JR messy decls, but works under openCV 3.4
string ServoAbs( double DEGRx,double DEGRy,double DEGLx,double DEGLy,double DEGNeck);
string ServoRel(double DEGRx,double DEGRy,double DEGLx,double DEGLy,double DEGNeck);
string TrackCorrelTarget (OwlCorrel OWL);
int OwlCalCapture(cv::VideoCapture &cap, string Folder);


//void ServoAbs(float DEGRx,float DEGRy,float DEGLx,float DEGLy,float DEGNeck);
//void ServoRel(float DEGRx,float DEGRy,float DEGLx,float DEGLy,float DEGNeck);
Mat DoGFilter(Mat src, int k, int g);

static string PiADDR = "10.0.0.10";
static int PORT=12345;
static SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);
static ostringstream CMDstream; // string packet
static string CMD;

//Default feature map weights
static int ColourWeight    = 60 ; //Saturation and Brightness
static int DoGHighWeight    = 60 ; //Groups of edges in a small area
static int DoGLowWeight     = 30 ; //DoG edge detection
static int FamiliarWeight = 5  ; //Familiarity of the target, how much has the owl focused on this before
static int foveaWeight    = 50 ; //Distance from fovea (center)


int main(int argc, char *argv[])
{
    //==========================================Initialize Variables=================================
    //Frame Size
    Size imageSize;
    imageSize.width=640;
    imageSize.height=480;

    //Local Feature Map  - implements FOVEA as a bias to the saliency map to central targets, rather than peripheral targets
    // eg. for a primate vision system
    Mat fovea=Mat(480, 640, CV_8U, double(0));
    fovea=Mat(480, 640, CV_8U, double(0));
    circle(fovea, Point(320,240), 150, 255, -1);
    cv::blur(fovea, fovea, Size(301,301));
    fovea.convertTo(fovea, CV_32FC1);
    fovea*=foveaWeight;

    //Initilize Mats
    const Mat OWLresult;// correlation result passed back from matchtemplate
    Mat Frame;
    Mat Left, Right, OWLtempl; // images
    Mat PanView=Mat(1600, 2500, CV_8UC3, Scalar(0,0,0)); //init as black - ie. no data
    Mat familiar=Mat(1600, 2500, CV_8U, double(255));

    //Video stream source
    string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name

    //Set center neck positions
//    Rx = RxC; Lx = LxC;
//    Ry = RyC; Ly = LyC;
//    Neck= NeckC;
    send_servos_home();

    //Variable declorations
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point minLocTarget; Point maxLocTarget;


    //===================================Read left calibration data===================================
    const string LeftCalibrationFile = argc > 1 ? argv[1] : "../../Data/LeftCalibration.xml";
    FileStorage LeftFile(LeftCalibrationFile, FileStorage::READ); // Read the settings
    if (!LeftFile.isOpened())
    {
        cout << "Cannot open LeftCalibration.xml: \"" << LeftCalibrationFile << "\"" << endl;
        return -1;
    }
    Mat cameraMatrixL, distCoeffsL;
    LeftFile["Camera_Matrix"] >> cameraMatrixL;
    LeftFile["Distortion_Coefficients"] >> distCoeffsL;
    LeftFile.release();

    Mat map1L, map2L;
    initUndistortRectifyMap(cameraMatrixL, distCoeffsL, Mat(),
    cv::getOptimalNewCameraMatrix(cameraMatrixL, distCoeffsL, imageSize, 1, imageSize, nullptr),
    imageSize, CV_16SC2, map1L, map2L);

    //===================================Read Right calibration data===================================
    const string RightCalibrationFile = argc > 1 ? argv[1] : "../../Data/RightCalibration.xml";
    FileStorage RightFile(RightCalibrationFile, FileStorage::READ); // Read the settings
    if (!RightFile.isOpened())
    {
        cout << "Cannot open RightCalibration.xml: \"" << RightCalibrationFile << "\"" << endl;
        return -1;
    }
    Mat cameraMatrixR, distCoeffsR;
    RightFile["Camera_Matrix"] >> cameraMatrixR;
    RightFile["Distortion_Coefficients"] >> distCoeffsR;
    RightFile.release();

    Mat map1R, map2R;
    initUndistortRectifyMap(cameraMatrixR, distCoeffsR, Mat(),
    getOptimalNewCameraMatrix(cameraMatrixR, distCoeffsR, imageSize, 1, imageSize, nullptr),
    imageSize, CV_16SC2, map1R, map2R);

    //========================================Initialize Servos========================================
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    //========================================Open Video Steam========================================
    VideoCapture cap (source);
    if (!cap.isOpened()){
        cout  << "Could not open the input video: " << source << endl;
        return -1;
    }

    while (1){//Main processing loop
        //cout<<"Capture Frame"<<endl;
        //==========================================Capture Frame============================================
        //for(int f=0;f<15;++f){
            if (!cap.read(Frame)){
                cout  << "Could not open the input video: " << source << endl;
            }
        //}

        Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1);     // Note that Left/Right are reversed now
        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        Left= FrameFlpd( Rect(0, 0, 640, 480));         // using a rectangle
        remap(Left, Left, map1L, map2L, INTER_LINEAR);  //Apply camera calibration
        Right=FrameFlpd( Rect(640, 0, 640, 480));       // using a rectangle
        remap(Right, Right, map1R, map2R, INTER_LINEAR);//Apply camera calibration



        // ======================================CALCULATE FEATURE MAPS ====================================



        /*-------- DoG low bandpass Map --------*/
        Mat LeftGrey;
        Mat LeftCpy = Left.clone();
        cvtColor(Left, LeftGrey, COLOR_BGR2GRAY);

        Mat DoGLow = DoGFilter(LeftGrey,3,51);
        Mat DoGLow8;
        normalize(DoGLow, DoGLow8, 0, 255, CV_MINMAX, CV_8U);
        //imshow("DoG Low", DoGLow8);

        /*-------- Detect High-Saturation --------*/
        Mat HSV, Hue, Saturation, Value, output;
        vector<Mat> hsv_channels;
        cvtColor(Left, HSV, CV_BGR2HSV);
        split(HSV, hsv_channels);

        Hue = hsv_channels[0];
        Saturation = hsv_channels[1];
        Value = hsv_channels[2];

        imshow("Hue", hsv_channels[0]);
        imshow("Saturation", hsv_channels[1]);
        imshow("Value", hsv_channels[2]);

        bitwise_and(hsv_channels[1], hsv_channels[2], output);

        imshow("Saturation * Value", output);

        /*---- Extract regions of color brightness above a threshold ----*/
        Mat BGR_thresh;
        double thresh = 127;
        threshold(Left, BGR_thresh,thresh,255, THRESH_BINARY);
        cvtColor(BGR_thresh,BGR_thresh, COLOR_BGR2GRAY);
       // imshow("RGB thresholding", BGR_thresh);



        //=====================================Initialise Global Position====================================
        //cout<<"Globe Pos"<<endl;
        Point GlobalPos;    // Position of camera view within the range of movement of the OWL
        GlobalPos.x=static_cast<int>(900+((-(Neck-NeckC)+(Lx-LxC))/DEG2PWM)/PX2DEG);
        GlobalPos.y=static_cast<int>(500+((Ly-LyC)/DEG2PWM)/PX2DEG);

        Mat familiarLocal=familiar(Rect(GlobalPos.x,GlobalPos.y,Left.cols,Left.rows));
        //imshow("familiarLocal",familiarLocal);

        //====================================Combine maps into saliency map=====================================
        //cout<<"Salience"<<endl;


        /*---- Convert 8-bit Mat to 32bit floating point ----*/

        //DoG
        DoGLow.convertTo(DoGLow, CV_32FC1);
        DoGLow*=DoGLowWeight;

        //Hue
        Hue.convertTo(Hue, CV_32FC1);
        Hue*=ColourWeight;

        //Saturation
        Saturation.convertTo(Saturation, CV_32FC1);
        Saturation*=ColourWeight;

        //BGR brightness
        BGR_thresh.convertTo(BGR_thresh, CV_32FC1);
        BGR_thresh*=ColourWeight;

        familiarLocal.convertTo(familiarLocal, CV_32FC1);

        // Linear combination of feature maps to create a salience map
        Mat Salience=cv::Mat(Left.size(),CV_32FC1,0.0); // init map

        add(Salience,DoGLow,Salience);
        //add(Salience,Hue, Salience);
        //add(Salience,Saturation, Salience);
        //add(Salience, BGR_thresh, Salience);

        add(Salience,fovea,Salience);

        Salience=Salience.mul(familiarLocal);
        normalize(Salience, Salience, 0, 255, CV_MINMAX, CV_32FC1);

        imshow("SalienceNew",Salience);

        //=====================================Find & Move to Most Salient Target=========================================

        minMaxLoc(Salience, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
        // Calculate relative servo correction and magnitude of correction
        double xDifference = static_cast<double>((maxLoc.x-320)*PX2DEG);
        double yDifference = static_cast<double>((maxLoc.y-240)*PX2DEG);

        rectangle(Left,Point(maxLoc.x-32,maxLoc.y-32),Point(maxLoc.x+32,maxLoc.y+32),Scalar::all(255),2,8,0); //draw rectangle on most salient area
        // Move left eye based on salience, move right eye to be parallel with left eye


        /* DISABLE FOR DEBUG */
        //ServoRel(((Lx-LxC+RxC-Rx)/DEG2PWM)+xDifference*1,-((LyC-Ly+RyC-Ry)/DEG2PWM)+yDifference*1,xDifference*1,yDifference*1,(Lx-LxC)/100);

        // Update Familarity Map //
        // Familiar map to inhibit salient targets once observed (this is a global map)
        double longitude=(((Ly-LyC)/DEG2PWM)+maxLoc.y*PX2DEG);//calculate longitude as the global map is a spherical projection
        // ensure dwell time at perimeter of map is similar to that at centre.
        longitude*=2.5; //amplify the projection correction
        if(longitude>70){
            longitude=70;
        }

        Mat familiarNew=familiar.clone();
        circle(familiarNew, GlobalPos+maxLoc, static_cast<int>(60/cos(longitude*PI/180)), 0, -1);
        cv::blur(familiarNew, familiarNew, Size(151,151)); //Blur used to save on processing
        normalize(familiarNew, familiarNew, 0, 255, CV_MINMAX, CV_8U);
        addWeighted(familiarNew, (static_cast<double>(FamiliarWeight)/100), familiar, (100-static_cast<double>(FamiliarWeight))/100, 0, familiar);

        Mat familiarSmall;
        resize(familiar,familiarSmall,familiar.size()/4);
        //imshow("Familiar",familiarSmall);
        //imshow("Right",Right);

        //=================================Convert Saliency into Heat Map=====================================

        Mat SalienceHSVnorm;
        Salience.convertTo(Salience, CV_8UC1);
        normalize(Salience, SalienceHSVnorm, 130, 255, CV_MINMAX, CV_8U);
        normalize(Salience, Salience, 0, 255, CV_MINMAX, CV_8U);
        Mat SalienceHSV;
        cvtColor(Left, SalienceHSV, COLOR_BGR2HSV);

        for(int y=0;y<480;y++){
            for(int x=0; x<640;x++){
                SalienceHSV.at<Vec3b>(y,x)=Vec3b(255-SalienceHSVnorm.at<uchar>(y,x),255,255);
            }
        }
        cvtColor(SalienceHSV, SalienceHSV, COLOR_HSV2BGR);


        //=======================================Update Global View===========================================
        //cout<<"Global View"<<endl;
        if(GlobalPos!=Point(0,0)){
            Mat LeftCrop = Left(Rect(220,140,200,200));//image cropped to minimize image stitching artifacts
            LeftCrop.copyTo(PanView(Rect(GlobalPos.x,GlobalPos.y,LeftCrop.cols,LeftCrop.rows)));
            Mat PanViewSmall;
            resize(PanView,PanViewSmall,PanView.size()/2);
            //imshow("PanView",PanViewSmall);
        }

        resize(Left,Left,Left.size()/2);
        imshow("Left",Left);
        resize(SalienceHSV,SalienceHSV,SalienceHSV.size()/2);
       // imshow("SalienceHSV",SalienceHSV);

        //=========================================Control Window for feature weights =============================================
        //cout<<"Control Window"<<endl;
        namedWindow("Control", CV_WINDOW_AUTOSIZE);
        cvCreateTrackbar("LowFreq"    , "Control", &DoGLowWeight    , 100);
        cvCreateTrackbar("FamiliarW", "Control", &FamiliarWeight, 100);
        cvCreateTrackbar("foveaW"   , "Control", &foveaWeight   , 100);

        waitKey(10);
    }
}

//====================================================================================//
// SERVO FUNCTIONS

string ServoAbs( double DEGRx,double DEGRy,double DEGLx,double DEGLy,double DEGNeck){
    int Rx,Ry,Lx,Ly, Neck;
    //    Rx=static_cast<int>(DEGRx*DEG2PWM)+RxC;
    //    Ry=static_cast<int>(-DEGRy*DEG2PWM)+RyC;
    //    Lx=static_cast<int>(DEGLx*DEG2PWM)+LxC;
    //    Ly=static_cast<int>(DEGLy*DEG2PWM)+LyC;
    //    Neck=static_cast<int>(-DEGNeck*DEG2PWM)+NeckC;
    Rx=static_cast<int>(DEGRx*DEG2PWM);
    Ry=static_cast<int>(DEGRy*DEG2PWM);
    Lx=static_cast<int>(DEGLx*DEG2PWM);
    Ly=static_cast<int>(DEGLy*DEG2PWM);
    Neck=static_cast<int>(DEGNeck*DEG2PWM);

    if(Rx>RxRm){
        Rx=RxRm;
    }else if(Rx<RxLm){
        Rx=RxLm;
    }

    if(Lx>LxRm){
        Lx=LxRm;
    }else if(Lx<LxLm){
        Lx=LxLm;
    }

    if(Ry>RyTm){
        Ry=RyTm;
    }else if(Ry<RyBm){
        Ry=RyBm;
    }

    if(Ly>LyBm){
        Ly=LyBm;
    }else if(Ly<LyTm){
        Ly=LyTm;
    }

    if(Neck>NeckL){
        Neck=NeckL;
    }else if(Neck<NeckR){
        Neck=NeckR;
    }

    servo_boundary_enforcer();

    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    string CMD = CMDstream.str();
    string retSTR = OwlSendPacket (u_sock, CMD.c_str());
    return (retSTR);
}

string ServoRel(double DEGRx,double DEGRy,double DEGLx,double DEGLy,double DEGNeck){
    //int Rx,Ry,Lx,Ly, Neck;
    Rx=static_cast<int>(DEGRx*DEG2PWM)+Rx;
    Ry=static_cast<int>(-DEGRy*DEG2PWM)+Ry;
    Lx=static_cast<int>(DEGLx*DEG2PWM)+Lx;
    Ly=static_cast<int>(DEGLy*DEG2PWM)+Ly;
    Neck=static_cast<int>(-DEGNeck*DEG2PWM)+Neck;

    if(Rx>RxRm){
        Rx=RxRm;
    }else if(Rx<RxLm){
        Rx=RxLm;
    }

    if(Lx>LxRm){
        Lx=LxRm;
    }else if(Lx<LxLm){
        Lx=LxLm;
    }

    if(Ry>RyTm){
        Ry=RyTm;
    }else if(Ry<RyBm){
        Ry=RyBm;
    }

    if(Ly>LyBm){
        Ly=LyBm;
    }else if(Ly<LyTm){
        Ly=LyTm;
    }

    if(Neck>NeckL){
        Neck=NeckL;
    }else if(Neck<NeckR){
        Neck=NeckR;
    }

    servo_boundary_enforcer();

    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    string CMD = CMDstream.str();
    string retSTR=OwlSendPacket (u_sock, CMD.c_str());
    return(retSTR);
}

string TrackCorrelTarget (OwlCorrel OWL){
    ostringstream CMDstream;
    string CMD, RxPacket;

    // Only for left eye at the moment
    //** P control set track rate to 10% of destination PWMs to avoid ringing in eye servo
    double KPx=0.1; // track rate X
    double KPy=0.1; // track rate Y
    double LxScaleV = LxRangeV/static_cast<double>(640); //PWM range /pixel range
    double Xoff= 320-(OWL.Match.x + OWLtempl.cols/2)/LxScaleV ; // compare to centre of image
    double LxOld=Lx;
    Lx=static_cast<int>(LxOld-Xoff*KPx); // roughly 300 servo offset = 320 [pixel offset]

    double LyScaleV = LyRangeV/static_cast<double>(480); //PWM range /pixel range
    double Yoff= (240+(OWL.Match.y + OWLtempl.rows/2)/LyScaleV)*KPy ; // compare to centre of image
    double LyOld=Ly;
    Ly=static_cast<int>(Yoff-LyOld); // roughly 300 servo offset = 320 [pixel offset]

    //cout << owl::Lx << " " << Xoff << " " << LxOld << endl; // DEBUG PFC
    //cout << owl::Ly << " " << Yoff << " " << LyOld << endl;

    //** ACTION
    // move to get minimise distance from centre of both images, ie verge in to target
    // move servos to position
    string retSTR=ServoAbs( ((RxC-Rx)/DEG2PWM), //Rx
             ((RyC-Ry)/DEG2PWM), // Ry -- the right eye Y servo has inverted direction compared to left.
             ((LxC-Lx)/DEG2PWM),// Lx
             ((LyC-Ly)/DEG2PWM), // Ly
             NeckC/DEG2PWM); // NECK .. no neck motion as yet
    return(retSTR);

}



// create DoG bandpass filter, with g being odd always and above 91 for low pass, and >9 for high pass
// k is normally 3 or 5
Mat DoGFilter(Mat src, int k, int g){
    Mat srcC;
    src.convertTo(srcC,CV_32FC1);
    Mat g1, g2;
    GaussianBlur(srcC, g1, Size(g,g), 0);
    GaussianBlur(srcC, g2, Size(g*k,g*k), 0);
    srcC = (g1 - g2)*2;
    return srcC;

}




















