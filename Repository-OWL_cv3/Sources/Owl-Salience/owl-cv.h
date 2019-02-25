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
Rect target = Rect(320-32, 240-32, 64, 64); // target is at the centre of the camera FOV
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
normalize( OWL.Result, OWL.Result);
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
