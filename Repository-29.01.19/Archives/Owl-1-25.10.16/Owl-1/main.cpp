// owl.cpp : Defines the entry point for the console application.
/* Phil Culverhouse Oct 2016 (c) Plymouth UNiversity
 *
 * Uses IP sockets to communicate to the owl robot (see owl-comms.h)
 * Uses OpenCV to perform normalised cross correlation to find a match to a template
 * (see owl-cv.h).
 * PWM definitions for the owl servos are held in owl-pwm.h
 * includes bounds check definitions
 * requires setting for specific robot
 *
 * This demosntration programs does the following:
 * a) loop 1 - take picture, check arrow keys
 *             move servos +5 pwm units for each loop
 *             draw 64x64 pixel square overlaid on Right image
 *             if 'c' is pressed copy patch into a template for matching with left
 *              exit loop 1;
 * b) loop 2 - perform Normalised Cross Correlation between template and left image
 *             move Left eye to centre on best match with template
 *             (treats Right eye are dominate in this example).
 *             loop
 *             on exit by ESC key
 *                  go back to loop 1
 *
 * First start communcations on Pi by running 'python PFCpacket.py'
 * Then run this program. The Pi server prints out [Rx Ry Lx Ly] pwm values and loops
 *
 * NOTE: this program is just a demonstrator, the right eye does not track, just the left.
 */

#include <iostream>
#include <fstream>

#include <sys/types.h>
#include <unistd.h>

#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"


#include <iostream> // for standard I/O
#include <string>   // for strings


using namespace std;
using namespace cv;



int main(int argc, char *argv[])
{
    char receivedStr[1024];
    ostringstream CMDstream; // string packet
    string CMD;
    int N;

    Rx = RxLm; Lx = LxLm;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name
    string PiADDR = "10.0.0.10";

    //SETUP TCP COMMS
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);

    /***********************
 * LOOP continuously for testing
 */
    // RyC=RyC-40; LyC=LyC+40; // offset for cross on card
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    const Mat OWLresult;// correlation result passed back from matchtemplate
    cv::Mat Frame;
    Mat Left, Right; // images
    bool inLOOP=true; // run through cursor control first, capture a target then exit loop

    while (inLOOP){
        // move servos to centre of field
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

        VideoCapture cap (source);              // Open input
        if (!cap.isOpened())
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }
        //Rect region_of_interest = Rect(x, y, w, h);
        while (inLOOP){
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                //         break;
            }
            Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
            //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Left= FrameFlpd( Rect(0, 0, 640, 480)); // using a rectangle
            Right=FrameFlpd( Rect(640, 0, 640, 480)); // using a rectangle
            Mat RightCopy;
            Right.copyTo(RightCopy);
            rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 ); // draw white rect
            imshow("Left",Left);imshow("Right", RightCopy);
            waitKey(10); // display the images
            int key = waitKey(0); // this is a pause long enough to allow a stable photo to be taken.
            switch (key){
            case 2490368: //up arrow
                Ry=Ry+5;Ly=Ly+5;
                break;
            case 2621440: //down arrow
                Ry=Ry-5;Ly=Ly-5;
                break;
            case 2424832: //left arrow
                Rx=Rx-5;Lx=Lx-5;
                break;
            case 2555904: // right arrow
                Rx=Rx+5;Lx=Lx+5;
                break;
            case 99: // lowercase 'c'
                OWLtempl= Right(target);
                imshow("templ",OWLtempl);
                waitKey(1);
                inLOOP=false; // quit loop and start tracking target
                break; // left
            default:
                key=key;
                //nothing at present
            }

                CMDstream.str("");
                CMDstream.clear();
                CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                CMD = CMDstream.str();
                RxPacket= OwlSendPacket (u_sock, CMD.c_str());

                if (0) {
                    for (int i=0;i<10;i++){
                        Rx=Rx-50; Lx=Lx-50;
                        CMDstream.str("");
                        CMDstream.clear();
                        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                        CMD = CMDstream.str();
                        RxPacket= OwlSendPacket (u_sock, CMD.c_str());
                        //waitKey(100); // cut the pause for a smooth persuit camera motion
                    }
                }
            } // END cursor control loop
            // close windows down
            destroyAllWindows();
            // just a ZMCC
            // right is the template, just captured manually
            inLOOP=true; // run through the loop until decided to exit
            while (inLOOP) {
                if (!cap.read(Frame))
                {
                    cout  << "Could not open the input video: " << source << endl;
                    break;
                }
                Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
                //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
                // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
                Left= FrameFlpd( Rect(0, 0, 640, 480)); // using a rectangle
                Right=FrameFlpd( Rect(640, 0, 640, 480)); // using a rectangle

                //Rect target= Rect(320-32, 240-32, 64, 64); //defined in owl-cv.h
                //Mat OWLtempl(Right, target);
                OwlCorrel OWL;
                OWL = Owl_matchTemplate( Right,  Left, OWLtempl, target);
                /// Show me what you got
                Mat RightCopy;
                Right.copyTo(RightCopy);
                rectangle( RightCopy, target, Scalar::all(255), 2, 8, 0 );
                rectangle( Left, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );
                rectangle( OWLresult, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );

                imshow("Owl-L", Left);
                imshow("Owl-R", RightCopy);
                imshow("Correl",OWL.Result );
                if (waitKey(10)== 27) inLOOP=false;
// P control
                double KPx=0.1; // track rate X
                double KPy=0.1; // track rate Y
                double LxScaleV = LxRangeV/(double)640; //PWM range /pixel range
                double Xoff= 320-(OWL.Match.x + OWLtempl.cols)/LxScaleV ; // compare to centre of image
                int LxOld=Lx;

                Lx=LxOld-Xoff*KPx; // roughly 300 servo offset = 320 [pixel offset


                double LyScaleV = LyRangeV/(double)480; //PWM range /pixel range
                double Yoff= (250+(OWL.Match.y + OWLtempl.rows)/LyScaleV)*KPy ; // compare to centre of image
                int LyOld=Ly;
                Ly=LyOld-Yoff; // roughly 300 servo offset = 320 [pixel offset

                cout << Lx << " " << Xoff << " " << LxOld << endl;
                cout << Ly << " " << Yoff << " " << LyOld << endl;
                //Atrous

                //Maxima

                // Align cameras

                // ZMCC disparity map

                // ACTION

                // move to get minimise distance from centre of both images, ie verge in to targe
                // move servos to position
                CMDstream.str("");
                CMDstream.clear();
                CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
                CMD = CMDstream.str();
                RxPacket= OwlSendPacket (u_sock, CMD.c_str());


            } // end if ZMCC
        } // end while outer loop
#ifdef __WIN32__
        closesocket(u_sock);
#else
        close(clientSock);
#endif
        exit(0); // exit here for servo testing only
    }
