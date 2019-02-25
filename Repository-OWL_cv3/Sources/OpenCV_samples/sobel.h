#ifndef SOBEL_H
#define SOBEL_H

// PFC adapted from http://docs.opencv.org/3.2.0/d2/d2c/tutorial_sobel_derivatives.html
// April 2017

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>

using namespace std;
using namespace cv;

int DOsobel(cv::Mat src, cv::Mat grad, cv::Mat phase )
{
    int scale = 1;
    int delta = 0;
    int ddepth = CV_64F;
    cv::Mat g8, p8;
    cv::Mat Igray;
  GaussianBlur( src, src, Size(31,31), 0, 0, BORDER_DEFAULT );
  cvtColor( src, Igray, COLOR_BGR2GRAY );
//cv::    imwrite("/Users/culverhouse/Teaching/AINT308/OWL-lectures/images/NingNing-s-gblur31.jpg",Igray);

  Mat gradx, grady;
  Mat abs_gradx, abs_grady;
  Sobel( Igray, gradx, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
  Sobel( Igray, grady, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( gradx, abs_gradx );
  convertScaleAbs( grady, abs_grady );
  addWeighted( abs_gradx, 0.5, abs_grady, 0.5, 0, grad, ddepth );
    
  // phase=atan2(grad_y,grad_x);

  for(int i = 0; i < gradx.rows; i++){
      for(int j = 0; j < gradx.cols; j++){
          // run through pixels, cannot do this as a MAT op.
          double Gx = gradx.at<double>(i,j);
          double Gy = grady.at<double>(i,j);
          double result = fastAtan2(Gy,Gx); // result in radians NOTE
          phase.at<double>(i,j) = result;
      }
   }
  //
  grad.convertTo(g8,CV_8U, 5,0); // scale by 5.0 to make bright (done on a DOUBLE array so safe)
  phase.convertTo(p8,CV_8U,5,0);
  cv::Mat g8s, p8s;
  cv::resize(g8, g8s, cv::Size(g8.cols * 0.5,g8.rows * 0.5), 0, 0, CV_INTER_LINEAR);
  cv::resize(p8, p8s, cv::Size(g8.cols * 0.5,g8.rows * 0.5), 0, 0, CV_INTER_LINEAR);
  cv::imshow( "Grad", g8s );
  cv::imshow( "Phase", p8s );
  char key=waitKey(30);
    if (key=='q') return (-1);
    else
        return(0);
}


#endif // SOBEL_H
