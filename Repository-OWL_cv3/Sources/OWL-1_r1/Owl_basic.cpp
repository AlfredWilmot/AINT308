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
#ifndef _WIN32
#include <unistd.h>
#endif
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

    //SETUP TCP COMMS
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);

    //centre eyes and neck
    Rx = RxC; Lx = 1470; // LxC;
    Ry = RyC; Ly = 1660; //LyC;
    Neck= NeckC;

    bool inLOOP=true; // run through cursor control first, capture a target then exit loop

    while (inLOOP){

        /*----------------------------------------------------------------------*/
        /* DO NOT REMOVE THIS BLOCK, OTHERWISE WILL BREAK */
        /*----------------------------------------------------------------------*/
        CMDstream.str("");
        CMDstream.clear();
        CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
        CMD = CMDstream.str();
        string RxPacket= OwlSendPacket (u_sock, CMD.c_str());
        /*----------------------------------------------------------------------*/

        /* Attempt to open video-feed */
        VideoCapture cap (source);
        if (!cap.isOpened())
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }

        while (inLOOP){

            // Capture frames and return any key values pressed.
            int key = camera_loop(&cap);

            switch (key){
            case 'w': //up
                Ry=Ry+5;Ly=Ly-5;
                break;
            case 's'://down
                Ry=Ry-5;Ly=Ly+5;
                break;
            case 'a'://left
                Rx=Rx-5;Lx=Lx-5;
                break;
            case 'd'://right
                Rx=Rx+5;Lx=Lx+5;
                break;
            case 'c':
                OWLtempl= Right(target);
                imshow("templ",OWLtempl);
                waitKey(1);
                inLOOP=false; // quit loop and start tracking target
                break;

            case 'j': // take image for calibration - max = 20
                if(calibCounter == 20){
                    cout << "20 pairs captured already" << endl;
                }else
                {
                    captureCalibPair(cap, myCalibrations, calibCounter);
                    calibCounter++;
                }
                break;

            case 27: //ESC
                inLOOP = false;
                break;
            default:
                key=key;
            }

            //============= Normalised Cross Correlation ==========================
            // right is the template, just captured manually

            if (start_cross_correlation) {

                /// P control for Left servo
                //** P control set track rate to 10% of destination PWMs to avoid ringing in eye servo
                //======== try altering KPx & KPy to see the settling time/overshoot
                double KPx=0.05; // track rate X
                double KPy=0.05; // track rate Y

                double LxScaleV = LxRangeV/static_cast<double>(640); //PWM range /pixel range
                double Xoff= 320-(OWL_left_eye.Match.x + OWLtempl.cols/2)/LxScaleV ; // compare to centre of image
                double LxOld=Lx;
                Lx=static_cast<int>(LxOld-Xoff*KPx); // roughly 300 servo offset = 320 [pixel offset]

                double LyScaleV = LyRangeV/static_cast<double>(480); //PWM range /pixel range
                double Yoff= (240+(OWL_left_eye.Match.y + OWLtempl.rows/2)/LyScaleV)*KPy ; // compare to centre of image
                double LyOld=Ly;
                Ly=static_cast<int>(LyOld-Yoff); // roughly 300 servo offset = 320 [pixel offset]
            }

            /* Update servo position */
            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();


#ifdef _WIN32
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());
#else
            OwlSendPacket (clientSock, CMD.c_str());
#endif


        } // end if ZMCC
    } // end while outer loop
#ifdef _WIN32
    closesocket(u_sock);
#else
    close(clientSock);
#endif
    exit(0); // exit here for servo testing only
}
