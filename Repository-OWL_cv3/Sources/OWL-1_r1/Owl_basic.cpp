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
#include "owl-stereo-calib.h"
#include "owl-disparity.h"


#include "distance_estimation.h"

#include <iostream> // for standard I/O
#include <string>   // for strings


using namespace std;
using namespace cv;


void servo_P_controller(double, double, int *, int *, OwlCorrel *, int *, int *, bool, int, int, int, int);


int main(int argc, char *argv[])
{
    char receivedStr[1024];
    ostringstream CMDstream; // string packet
    string CMD;
    int N;

    //SETUP TCP COMMS
    int PORT=12345;
    SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);

    /* Center servo values (according to owl #8 servo calibration) */
    Rx   = Owl_8_RxC; // - 18*2;
    Lx   = Owl_8_LxC; //+ 18*2;
    Ry   = Owl_8_RyC;
    Ly   = Owl_8_LyC;
    Neck = Owl_8_NeckC;

//    /* Toe in servo values (according to owl #1 servo calibration) */
//    Rx   = Owl_1_RxC - 18*3;    //toe-in of roughly 6 degrees.
//    Lx   = Owl_1_LxC + 18*3;
//    Ry   = Owl_1_RyC;
//    Ly   = Owl_1_LyC;
//    Neck = Owl_1_NeckC;

    bool inLOOP=true; // run through cursor control first, capture a target then exit loop
    bool canCalib = false;

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

            int step = 5;

            switch (key){
            case 'w': //up
                Ry=Ry+step;Ly=Ly-step;
                break;
            case 's'://down
                Ry=Ry-step;Ly=Ly+step;
                break;
            case 'a'://left
                Rx=Rx-step;Lx=Lx-step;
                break;
            case 'd'://right
                Rx=Rx+step;Lx=Lx+step;
                break;
            case 'c':
                OWLtempl= Right(target);
                imshow("templ",OWLtempl);
                waitKey(1);
                inLOOP=false; // quit loop and start tracking target
                break;
            case 'j': // take image for calibration - max = 20

                if(calibCounter < 20){
                    captureCalibPair(cap, testImages, calibCounter);
                    calibCounter++;

                }else
                {
                    canCalib = true;
                    cout << "20 pairs captured already" << endl;
                }
                break;
            case 'k': // toggles requirement for calibration pictures and print if true or false

                canCalib = !canCalib;

                cout << "\nCan calibrate = "
                     << boolalpha << canCalib
                     << endl;

                break;
            case 'l':

                /*/
                 *   Pre-calibration steps
                 * - Change folder location
                 * - Create the new folder for that location
                 * - Take pictures
                 * - Change .xml which calibration set to use in parser Test'N'
                 /*/

                // Currently using Mark calibrations set (test 1)

                destroyAllWindows(); //removes irrelevant windows
                if (canCalib)
                {
                    Size boardSize; //store board size parameters
                    string imagelistfn; //store image list locations
                    bool showRectified; //chose to show rectified images

                    cv::CommandLineParser parser(argc, argv, "{w|9|}{h|6|}{s|26.0|}{nr||}{help||}{@input|../../Data/stereo_calib_Test1.xml|}");
                    if (parser.has("help"))
                        return print_help();
                    showRectified = !parser.has("nr");
                    imagelistfn = parser.get<string>("@input");
                    boardSize.width = parser.get<int>("w");
                    boardSize.height = parser.get<int>("h");
                    float squareSize = parser.get<float>("s");
                    if (!parser.check())
                    {
                        parser.printErrors();
                        return 1;
                    }
                    vector<string> imagelist;
                    bool ok = readStringList(imagelistfn, imagelist);
                    if(!ok || imagelist.empty())
                    {
                        cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
                        return print_help();
                    }

                    StereoCalib(imagelist, boardSize, squareSize, true, true, showRectified);
                    canCalib = false;
                    camera_setup_done= false; //reset flag so that can use click-to-verge feature.
                    destroyAllWindows();
                }

                break;
            case 'g':

                destroyAllWindows(); //closes irrelevant windows
                showDisparity(argc - 1, argv + 1);
                camera_setup_done= false; //reset flag so that can use click-to-verge feature.
                destroyAllWindows();

                break;
            case 27: //ESC
                inLOOP = false;
                break;
            default:
                key=key;
                 /* DEBUG SERVO POSITION */
//                cout << "Rx:\t" << Rx << "\nLx:\t" << Lx << "\n";
//                cout << "Ry:\t" << Ry << "\nLy:\t" << Ly << "\n" << "Neck:\t" << Neck << "\n\n";
                update_Rx_theta();
                update_Lx_theta();
//                cout << "Rx_theta: " << Rx_theta << " deg\n";
//                cout << "Lx_theta: " << Lx_theta << " deg\n";

            }

            //============= Normalised Cross Correlation ==========================
            // right is the template, just captured manually

            if (start_cross_correlation) {

                /// P control for the servo
                //** P control set track rate to 10% of destination PWMs to avoid ringing in eye servo

                servo_P_controller(0.1, 0.1, &LxRangeV, &LyRangeV, &OWL_left_eye, &Lx, &Ly, false, Owl_1_LxMin, Owl_1_LxMax, Owl_1_LyMin, Owl_1_LxMax);
                servo_P_controller(0.1, 0.1, &RxRangeV, &RyRangeV, &OWL_right_eye, &Rx, &Ry, true, Owl_1_RxMin, Owl_1_RxMax, Owl_1_RyMin, Owl_1_RxMax);

                //update_distance_estimate();
                update_distance_estimate_PFC();

                //cout << "target distance: " << distance_estimate << "\n";
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



void servo_P_controller(double KPx, double KPy, int *range_x, int *range_y, OwlCorrel *owl_eye, int *axis_x, int *axis_y, bool is_right_eye, int min_x, int max_x, int min_y, int max_y)
{

    double tmp = 0;

    double xScaleV = *range_x/static_cast<double>(640);                    //PWM range /pixel range
    double Xoff= 320-(owl_eye->Match.x + OWLtempl.cols/2)/xScaleV ;        // compare to centre of image
    tmp = *axis_x;
    *axis_x=static_cast<int>(tmp-Xoff*KPx);                                 // roughly 300 servo offset = 320 [pixel offset]

    double yScaleV = *range_y/static_cast<double>(480);                    //PWM range /pixel range

    double Yoff = 0;

    if(is_right_eye)
    {
        Yoff = 240-(owl_eye->Match.y - OWLtempl.rows/2)/yScaleV ;  // compare to centre of image
        tmp = *axis_y;
        *axis_y=static_cast<int>(tmp + Yoff*KPy);// roughly 300 servo offset = 320 [pixel offset]
    }
    else
    {
        Yoff = 240+(owl_eye->Match.y + OWLtempl.rows/2)/yScaleV ;
        tmp = *axis_y;
        *axis_y=static_cast<int>(tmp - Yoff*KPy);// roughly 300 servo offset = 320 [pixel offset]
    }


    /* x-axis software end-stop */
    if((double)max_x <= *axis_x)
    {
        *axis_x = max_x;
    }
    else if ((double)min_x >= *axis_x)
    {
        *axis_x = min_x;
    }


    /* y-axis software end-stop */
    if((double)max_y <= *axis_y)
    {
        *axis_y = max_y;
    }
    else if ((double)min_y >= *axis_y)
    {
        *axis_y = min_y;
    }


}
//disp ushort for disparity not disp8
//find relationship between brightness and distance in disparity image. can be linear or inverse
