#ifndef OWLCV_H
#define OWLCV_H

#endif // OWLCV_H

/* Phil Culverhouse
 *
 * Vision Processing for OWL camera system
 *  Currently provides Normalised Cross Correlation for template match
 *  uses opencv, assumes 3.1 or similar
 *  uses the Right eye for template source.
 * (c) Plymouth University, 2016
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

struct OwlCorrel {
    Point Match;
    Mat Result;
};

Mat OWLtempl; // used in correlation

                             // drawn over whatever is in the centre of the FOV, to act as a template

struct OwlCorrel Owl_matchTemplate(Mat Right, Mat Left, Mat templ, Rect target){


/// Create the result matrix
int result_cols =  Left.cols - templ.cols + 1;
int result_rows = Left.rows - templ.rows + 1;

static OwlCorrel OWL;
OWL.Result.create(result_rows, result_cols,  CV_32FC1 );

/// Do the Matching and Normalize
int match_method = 5; /// CV_TM_CCOEFF_NORMED;
matchTemplate( Left, templ, OWL.Result, match_method );
/// Localizing the best match with minMaxLoc
double minVal; double maxVal; Point minLoc; Point maxLoc;
Point matchLoc;

minMaxLoc( OWL.Result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
//if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED ) //CV3
if( match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED ) //CV4
{ OWL.Match = minLoc; }
else
{ OWL.Match = maxLoc; }

return (OWL);
}

int OwlCalCapture(cv::VideoCapture &cap, string Folder){

    int count=20;
    cv::Mat Frame;
    for (int i=0;i<count;i++){
        if (!cap.read(Frame))
        {
            return(-1);
        }
        //Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
        //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        cv::Mat Right= Frame( Rect(0, 0, 640, 480)); // using a rectangle
        cv::Mat Left=  Frame( Rect(640, 0, 640, 480)); // using a rectanglecv::imwrite(Folder + "left" + count + "jpg", Left);
        string fnameR(Folder + "right" + to_string(i) + ".jpg");
        string fnameL=(Folder + "left" +  to_string(i) + ".jpg");
        cv::imwrite(fnameL, Left);
        cv::imwrite(fnameR, Right);
        cout << "Saved " << i << " stereo pair" << Folder <<endl;
        cv::waitKey(0);
    }

    cout << "Just saved 10 stereo pairs" << Folder <<endl;
    return(0);
}


/*------------------------*/
/*-- Camera calibration --*/
/*------------------------*/

//calibration vars
static int calibCounter = 0;
const string myCalibrations = "../../Data/mySavedImages/Test"; //location of the folder to store calibrating images

//Take the
int captureCalibPair(cv::VideoCapture &cap, string Folder, int calibCounter){

cv::Mat Frame; // create matrix for the camera feed

    if (!cap.read(Frame))
    {
        return(-1);
    }
    //Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
    //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
    // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
    cv::Mat Right= Frame( Rect(0, 0, 640, 480)); //Take the right frame
    cv::Mat Left=  Frame( Rect(640, 0, 640, 480)); //Take the right frame u
    string fnameR(Folder + "right" + to_string(calibCounter) + ".jpg");
    string fnameL=(Folder + "left" +  to_string(calibCounter) + ".jpg");
    cv::imwrite(fnameL, Left);
    cv::imwrite(fnameR, Right);
    cout << "Saved " << calibCounter + 1 << " stereo pair" << Folder <<endl;
    cv::waitKey(100);

    calibCounter++;

}


/*--------------------*/
/*-- Vergence Code --*/
/*------------------*/

/*---- Camera variables ----*/
static bool _mouse_clk = false;
static bool camera_setup_done = false;

const   cv::Point mid_pxl    = cv::Point(320, 240);
static  cv::Point target_pxl = mid_pxl;

static cv::Rect target = Rect(target_pxl.x-32, target_pxl.y-32, 64, 64); // target is at the centre of the camera FOV

static  cv::Mat Left, Right; // images

const std::string right_eye = "TEST";
const std::string left_eye  = "Left_Eye";

const std::string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name
const std::string PiADDR = "10.0.0.10";

static bool inLOOP=true; // run through cursor control first, capture a target then exit loop

static bool start_cross_correlation = false; // determines if cross-correlation is to be performed on the target ROI or not.

/*---- Handler method that reacts to user selecting pixel in interactive window ----*/
void mouseEvent(int evt, int x, int y, int, void*)
{

    if (evt == CV_EVENT_LBUTTONDOWN)
    {
        _mouse_clk = true;      //set flag.

        /* Update the new mouse-selected seed pixel coordinates */
        target_pxl = cv::Point(x,y);

        std::cout << "Pixel (x,y): " << target_pxl.x << ", " << target_pxl.y << "\n";

        /* Phil's code for starting cross-correlation */

        target = Rect(target_pxl.x-32, target_pxl.y-32, 64, 64); // update target to anchor at selected pixel locatoin

        OWLtempl= Right(target);
        imshow("templ",OWLtempl);
        waitKey(1);

        /* Start tracking the ROI at the selected anchor using cross-correlation */
        start_cross_correlation = true;

    }
}

/*---- Loop for camera functionalituy ----*/
const cv::Mat OWLresult;// correlation result passed back from matchtemplate
static cv::Mat Frame;
static cv::VideoCapture cap;
int camera_loop(cv::VideoCapture *vid_cap)
{
    if (!vid_cap->read(Frame))
    {
        std::cout  << "Could not open the input video: " << source << std::endl;
        //         break;
        return -2;
    }
    cv::Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
    //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
    // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
    Left= FrameFlpd( cv::Rect(0, 0, 640, 480)); // using a rectangle
    Right=FrameFlpd( cv::Rect(640, 0, 640, 480)); // using a rectangle
    cv::Mat RightCopy;
    Right.copyTo(RightCopy);

    /* Draw stuff onto img, then show images */
    target = Rect(target_pxl.x-32, target_pxl.y-32, 64, 64);
    cv::rectangle( RightCopy, target, cv::Scalar::all(255), 2, 8, 0 ); // draw white rect
    if(_mouse_clk)cv::line(RightCopy, mid_pxl, target_pxl, cv::Scalar(0, 255, 0), 3); // draw line from center of screen to selected pixel location

    imshow(left_eye, Left);imshow(right_eye, RightCopy);

    /* Only set-up call-backs once the window is established */
    if(!camera_setup_done)
    {
        cv::setMouseCallback(right_eye, mouseEvent, 0);
        camera_setup_done = true;
    }


    cv::waitKey(10);

    return cv::waitKey(100); // this is a pause long enough to allow a stable photo to be taken.

}


/*---- Moves all servos to the position corresponding to the latest PWM mark-period value ----*/
//void update_servo_position(int ms_delay = 20)
//{
//    /* Construct servo control packet */
//    CMDstream.str("");
//    CMDstream.clear();
//    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
//    CMD = CMDstream.str();
//    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

//    cout << "\nRx:" << Rx << "\nRy:" << Ry << "\nLx:" << Lx << "\nLy:" << Ly << "\nNeck:"  << Neck << "\n";

//    /* Induce delay to give packet enought time to send */
//    waitKey(ms_delay);
//}
