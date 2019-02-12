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

#ifndef TEMPLATE_CODE_H
#define TEMPLATE_CODE_H

#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <iostream> // for standard I/O
#include <string>   // for strings

#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

char receivedStr[1024];
ostringstream CMDstream; // string packet
string CMD;
int N;

string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name
string PiADDR = "10.0.0.10";

//SETUP TCP COMMS
int PORT=12345;
SOCKET u_sock = OwlCommsInit ( PORT, PiADDR);

const cv::Mat OWLresult;// correlation result passed back from matchtemplate
cv::Mat Frame;
cv::Mat Left, Right; // images
bool inLOOP=true; // run through cursor control first, capture a target then exit loop

/*****************************************/
/********USER MADE FUNCTIONS**************/
/*****************************************/

/* Moves all servos to the position corresponding to the latest PWM mark-period value */
void update_servo_position(int ms_delay = 20)
{
    /* Construct servo control packet */
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    cout << "\nRx:" << Rx << "\nRy:" << Ry << "\nLx:" << Lx << "\nLy:" << Ly << "\nNeck:"  << Neck << "\n";

    /* Induce delay to give packet enought time to send */
    waitKey(ms_delay);
}

/* Defines the step that should be taken as a sinusoidal function of the current neck position */
int get_sin_step(int neck_val)
{
 //int step = int((sin(double(neck_val)/double(974)) - 0.904)*200) ;
 double calc_step = (sin(double(neck_val)/974) - 0.904)*800;

 cout << "Step size: " << calc_step;

 int step_out = int(round(calc_step));

 if (step_out <= 0)
 {
     return 10;
 }
 else
 {
    return step_out;
 }
}


/* global variables that keep track of the last known step-sizes of a given axis */
static int step_Rx   = -1;
static int step_Ry   = -1;
static int step_Lx   = -1;
static int step_Ly   = -1;
static int step_Neck = -1;

/* flags used to keep track of progress of each axis during actuation (allows for pseudo-concurrent movement of axes)*/
    int Rx_done    = 0;
    int Ry_done    = 0;
    int Lx_done    = 0;
    int Ly_done    = 0;
    int Neck_done  = 0;

#define pi 3.14159265358979323846

/* input position start & stop, and the axis to be actuated */
int sine_motion(double speed, int start, int stop, int *axis_ptr, int *axis_step_ptr)
{

    double x_axis_scaling = -pi/abs(start-stop);
    double step_scaling = abs(start-stop)*speed;

    int store_sine_output = 0;

    /* If the step has not been defined yet, move servo into position before taking any steps.
       Reset step value to -1 once the stop position is reached. */
    if(*axis_step_ptr == -1)
    {
        *axis_step_ptr = 0;
        *axis_ptr = start;
        update_servo_position();
    }

    /* store sine output */
    store_sine_output = int( round( (sin(*axis_step_ptr * x_axis_scaling + pi) + 0.1) * step_scaling ) );

    /* increment step */
    *axis_step_ptr += store_sine_output;

    /* positive rotation */
    if(start < stop)
    {
        *axis_ptr += store_sine_output;

        /* saturate joint value to stop position if it exceeeds it */
        //Rx >= stop ? Rx = stop : Rx = Rx;
        if(*axis_ptr >= stop)
        {
            *axis_ptr = stop;
            *axis_step_ptr = -1;
        }

    }
    /* negative rotation */
    else if(start > stop)
    {
        *axis_ptr -= store_sine_output;

        /* saturate joint value to stop position if it falls below it */
        //Rx <= stop ? Rx = stop : Rx = Rx;
        if(*axis_ptr <= stop)
        {
            *axis_ptr = stop;
            *axis_step_ptr = -1;
        }
    }
    else
    {
        cout << "Invalid input! Do nothing.\n";
    }

    //update_servo_position();

    cout << " Step size: " << store_sine_output << "\nCurrent step value: " << *axis_step_ptr << "\nCurrent axis value: " <<*axis_ptr << "\n\n";

    /* Finished moving from start to stop */
    if(*axis_step_ptr == -1)
    {
        /* return 1 to indicate finished moving along this axis */
        return 1;
    }

    /* indicate current axis has not yet reached it's destination */
    return 0;
}

/* Test script for driving neck in a sinusoidal motion (peak at centre) */
int sinusoidal_neck()
{
    /* (divide by 974, in order to get peak of sin to occur at "NeckC"=1530 i.e. 1530/974 = pi/2.) */
    /* start at center position. Go through one sinusoidal rotation: going from NeckC, NeckR, NeckL,NeckC. */

    Neck = NeckC;   // start at center position.
    update_servo_position();

    int state = 1;      // defines direction of travel (sinusoid output defines step-size, hence speed).

    while(1)
    {
        switch(state)
        {
            // (1) rotate from NeckC to NeckR.
            case 1:

                // Turning right
                Neck -= get_sin_step(Neck);
                //Neck -= 5;

                if(Neck <= NeckR)
                {
                    state = 2; //start turning left once min of range is reached.
                    Neck = NeckR;
                }

                update_servo_position();

                break;

            /* (2) rotate from NeckR to NeckL. */
            case 2:

                // Turning left
                Neck += get_sin_step(Neck);
                Neck += 5;

                if(Neck >= NeckL)
                {
                    state = 3; //start turning right once max of range is reached.
                    Neck = NeckL;
                }

                update_servo_position();

                break;

            /* (3) rotate from NeckL to NeckC. */
            case 3:

                // turning right
                Neck -=get_sin_step(Neck);
                //Neck -= 5;

                if(Neck <= NeckC)
                {
                    Neck = NeckC;
                    update_servo_position();
                    return 0;   //exit when done
                }

                update_servo_position();

                break;

            /* Something went wrong ...*/
            default:

                return -1;
        }


        /* Exit loop and safely deallocate memory for display windows if ESC key is pressed */
        if (waitKey(10) == 27)
        {
            destroyAllWindows();
            break;
        }
    }
}


/* Mimicking Chameleon eye movement from first ten seconds of video (https://www.youtube.com/watch?v=otmt6YUplDw) */
int Chameleon ()
{
    // 0) Right eye looking forward, left eye angled downwards.
    Rx = RxC;
    Ry = RyC;

    Lx = LxLm;
    Ly = LyBm + LyRangeM/4;

    update_servo_position();
    waitKey(980);

    // 1) Left eye slightly rotates towards the center and downwards.

    Lx += LxRangeM/4;
    Ly -= LyRangeM/4;

    update_servo_position();
    waitKey(980);

    // 2) Left eye rotates back to left-most position, but maintains same downward angle.
    Lx = LxLm;
    update_servo_position();
    waitKey(980);

    // 3) Right eye rotates slightly upwards, and left eye tilts upwards to look at horizon.
    Ry = RyC + RyRangeM/4;

    Ly = LyC;
    update_servo_position();
    waitKey(980);

    // 4) Right eye rotates down and towards the left, left eye rotates downwards.
    Ry = RyBm + RyRangeM/4;
    Rx = RxLm + RxRangeM/4;

    Ly = LyBm;

    update_servo_position();
    waitKey(980);

    // 5) Left eye goes back to center of y axis.
    Ly = LyC;

    update_servo_position();
    waitKey(980);

    // 6) Left eye looks all the way up.
    Ly = LyTm;

    update_servo_position();
    waitKey(980);

    // 7) Right eye rotates slightly towards the left and up, and left eye goes slightly towards the right and looks at horizon.
    Rx = RxLm;
    Ry = RyC;

    Ly = LyC;
    Lx = LxLm + LxRangeM/5;

    update_servo_position();
    waitKey(980);

    // 8) Right eye rotates to the right and looks slightly downwards; Left eye goes towards the left and slightly down.
    Rx = RxRm - RxRangeM/4;
    Ry = RyBm;

    Ly = LyC + LyRangeM/4;
    Lx = LxLm + LxRangeM/4;

    update_servo_position();
    waitKey(980);

    // 9) Right eye rotates all the way right and upwards to horizon, left eye rotates all the way down.
    Rx = RxRm;
    Ry = RyC;

    Ly = LyBm;

    update_servo_position();
    waitKey(980);

    return 0;
}

/* Recreating the very cheeky eye-roll displayed here: https://www.youtube.com/watch?v=x2W0tzz7fS8*/
int CheekyEyeRoll()
{
    /* First move eyes up and then around to the right (quickly), pause slightly, and slowly move neck to the right */
    int state = 0;

    while(state < 3)
    {
        switch (state) {

        /* Eyes up */
        case 0:

            /* Move L & R eye smoothly upwards simultaneously and then move to next state when they're both done */
            Ry_done == 0 ? Ry_done = sine_motion(1.0, RyC, RyTm, &Ry, &step_Ry) : Ry_done = Ry_done;
            Ly_done == 0 ? Ly_done = sine_motion(1.0, LyC, LyTm, &Ly, &step_Ly) : Ly_done = Ly_done;

            if(Ry_done && Ly_done)
            {
                state++;
                Ry_done = 0;
                Ly_done = 0;
            }

            break;

        /* Rotate eyes towards the right and down towards the horizon */
        case 1:

            /* Rotate both left and right eyes towards the right and the horizon */
            Rx_done == 0 ? Rx_done = sine_motion(0.1, RxC, RxRm, &Rx, &step_Rx) : Rx_done = Rx_done;
            Ry_done == 0 ? Ry_done = sine_motion(0.05, RyTm, RyC, &Ry, &step_Ry) : Ry_done = Ry_done;

            Lx_done == 0 ? Lx_done = sine_motion(0.1, LxC, LxRm, &Lx, &step_Lx) : Lx_done = Lx_done;
            Ly_done == 0 ? Ly_done = sine_motion(0.05, LyTm, LyC, &Ly, &step_Ly) : Ly_done = Ly_done;


            if(Ry_done && Ly_done && Rx_done && Lx_done)
            {
                /* Wait for dramatic effect! */
                waitKey(200);

                state++;
                Rx_done = 0;
                Lx_done = 0;
                Ry_done = 0;
                Ly_done = 0;
            }
            break;

        /* Slightly rotate neck towards direction of gaze */
        case 2:

            if(sine_motion(0.05, NeckC, NeckC - NeckRange/4, &Neck, &step_Neck) == 0)
            {
               update_servo_position();
            }
            else
            {
                /* Wait for dramatic effect! */
                waitKey(1000);

                state++;
            }

            break;

        default:
            break;
        }

        update_servo_position();
    }

    return 0;
}

/* Sheepish glance */
int SheepishGlance()
{
    int state = 0;

    while(state < 2)
    {
        switch (state) {

        /* Rotate Neck fully right (to face "target"). */
        case 0:

            Neck_done == 0 ? Neck_done = sine_motion(0.1, NeckC, NeckR, &Neck, &step_Neck) : Neck_done = Neck_done;

            if(Neck_done)
            {
                state++;
                Neck_done = 0;
            }
            break;

        /* Slowly rotate Neck to the left, whilst keeping the eyes on the target. */
        case 1:

            /* (Rx & Ry need to move x-times faster than neck to execute behaviour convincingly--
             *  x is roughly ratio between radii of neck and y-axes of eye servo) */
            Rx_done == 0    ? Rx_done = sine_motion(0.02, RxC, RxRm, &Rx, &step_Rx) : Rx_done = Rx_done;
            Lx_done == 0    ? Lx_done = sine_motion(0.02, LxC, LxRm, &Lx, &step_Lx) : Lx_done = Lx_done;

            Neck_done == 0  ? Neck_done = sine_motion(0.02, NeckR, NeckC, &Neck, &step_Neck) : Neck_done = Neck_done;

            if(Rx_done && Lx_done && Neck_done)
            {
                state++;
                Rx_done = 0;
                Lx_done = 0;
                Neck_done = 0;
            }
            break;

        default:
            break;
        }

        update_servo_position();
    }
    return 0;
}
/* Scanning horizon */
int ScanHorizon()
{

    int state = 0;
    while(state < 4)
    {

        switch (state) {

            /* Pan eyes from center to left */
            case 0:

                Rx_done == 0 ? Rx_done = sine_motion(0.1, RxC, RxLm, &Rx, &step_Rx) : Rx_done = Rx_done;
                Lx_done == 0 ? Lx_done = sine_motion(0.1, LxC, LxLm, &Lx, &step_Lx) : Lx_done = Lx_done;



                if(Rx_done && Lx_done)
                {
                    /* Wait for dramatic effect! */
                    waitKey(100);

                    state++;
                    Rx_done = 0;
                    Lx_done = 0;
                }
                break;

            /* Pan eyes from left to right */
            case 1:

                Rx_done == 0 ? Rx_done = sine_motion(0.1, RxLm, RxRm, &Rx, &step_Rx) : Rx_done = Rx_done;
                Lx_done == 0 ? Lx_done = sine_motion(0.1, LxLm, LxRm, &Lx, &step_Lx) : Lx_done = Lx_done;



                if(Rx_done && Lx_done)
                {
                    /* Wait for dramatic effect! */
                    waitKey(100);

                    state++;
                    Rx_done = 0;
                    Lx_done = 0;
                }
                break;

            /* Pan eyes from right to left  */
            case 2:
                Rx_done == 0 ? Rx_done = sine_motion(0.1, RxRm, RxLm, &Rx, &step_Rx) : Rx_done = Rx_done;
                Lx_done == 0 ? Lx_done = sine_motion(0.1, LxRm, LxLm, &Lx, &step_Lx) : Lx_done = Lx_done;



                if(Rx_done && Lx_done)
                {
                    /* Wait for dramatic effect! */
                    waitKey(100);

                    state++;
                    Rx_done = 0;
                    Lx_done = 0;
                }
                break;

            /* Pan eyes from left back to center */
            case 3:
                Rx_done == 0 ? Rx_done = sine_motion(0.1, RxLm, RxC, &Rx, &step_Rx) : Rx_done = Rx_done;
                Lx_done == 0 ? Lx_done = sine_motion(0.1, LxLm, LxC, &Lx, &step_Lx) : Lx_done = Lx_done;



                if(Rx_done && Lx_done)
                {
                    /* Wait for dramatic effect! */
                    waitKey(100);

                    state++;
                    Rx_done = 0;
                    Lx_done = 0;
                }
                break;

            default:
                break;
        }

        update_servo_position();

    }

    return 0;
}

/* Move servos to home position */
void reset_servos()
{
    /* Initializing servo positions */
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    update_servo_position();
}


/*****************************************/

int template_code_script()
{

    while (inLOOP)
    {
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



        reset_servos();

        //Rect region_of_interest = Rect(x, y, w, h);
        while (inLOOP)
        {
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


            switch (key)
            {
                case'w': /* UP */
                    Ry=Ry+5;Ly=Ly-5;
                    break;
                case's': /* DOWN */
                    Ry=Ry-5;Ly=Ly+5;
                    break;
                case'a': /* LEFT */
                    Rx=Rx-5;Lx=Lx-5;
                    break;
                case'd': /* RIGHT */
                    Rx=Rx+5;Lx=Lx+5;
                break;
                    case 'c': /* START TRACKING TARGET */
                    OWLtempl= Right(target);
                    imshow("templ",OWLtempl);
                    waitKey(1);
                    inLOOP=false; // quit loop and start tracking target
                    break; // left

                case '1':   /* SINUSOIDAL NECK DEMO */

                    while(sine_motion(0.05, NeckC, NeckL, &Neck, &step_Neck) == 0)
                    {
                        update_servo_position();
                    }
                    while(sine_motion(0.05, NeckL, NeckR, &Neck, &step_Neck) == 0)
                    {
                        update_servo_position();
                    }
                    while(sine_motion(0.05, NeckR, NeckL, &Neck, &step_Neck) == 0)
                    {
                        update_servo_position();
                    }
                    while(sine_motion(0.05, NeckL, NeckC, &Neck, &step_Neck) == 0)
                    {
                        update_servo_position();
                    }


                    break;

                case '2':   /* CHAMELEON DEMO */

                    Chameleon();
                    reset_servos();

                    break;

                case '3':   /* HUMAN BEHAVIOUR 1: EYE-ROLL */

                    CheekyEyeRoll();
                    reset_servos();

                    break;

                case '4':   /* HUMAN BEHAVIOUR 2: SHEEPISH-GLANCE */

                    SheepishGlance();
                    reset_servos();

                    break;

                case '5':   /* SCAN HORIZON: MIMIC HUMAN TARGET TRACKING */

                    ScanHorizon();
                    reset_servos();

                    break;

                            /* DO NOTHING BY DEFAULT */
                default:
                    key=key;

            }

            CMDstream.str("");
            CMDstream.clear();
            CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
            CMD = CMDstream.str();
            RxPacket= OwlSendPacket (u_sock, CMD.c_str());

            if (0)
            {
                for (int i=0;i<10;i++)
                {
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
            rectangle( OWL.Result, OWL.Match, Point( OWL.Match.x + OWLtempl.cols , OWL.Match.y + OWLtempl.rows), Scalar::all(255), 2, 8, 0 );

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

    return 0;
}


#endif
