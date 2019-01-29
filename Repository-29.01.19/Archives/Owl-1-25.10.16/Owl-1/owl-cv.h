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
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
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
normalize( OWL.Result, OWL.Result, 0, 1, NORM_MINMAX, -1, Mat() );
/// Localizing the best match with minMaxLoc
double minVal; double maxVal; Point minLoc; Point maxLoc;
Point matchLoc;

minMaxLoc( OWL.Result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
{ OWL.Match = minLoc; }
else
{ OWL.Match = maxLoc; }

return (OWL);
}
