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

#include "template_code.h"

using namespace std;
using namespace cv;

/* Defines the step that should be taken as a sinusoidal function of the current neck position */
int get_sin_step(int neck_val)
{
 int step = int( round( (sin(neck_val/974) - 0.904)*100 ) ) ;

 return step;

}

/* Test script for driving neck in a sinusoidal motion (peak at centre) */
void sinusoidal_neck()
{
    //divide by 974, in order to get peak of sin to occur at "NeckC"=1530


    float tmp = 0.0;    // holds sin-processed value.

    // start at center position. Go through one sinusoidal rotation: going from NeckC, NeckR, NeckL,NeckC.

    int neck = NeckC;   // start at center position.

    int state = 1;      // defines direction of travel (sinusoid defines step-size).


    for(int i = 1; i < 2*NeckRange; i++)
    {
        switch(state)
        {
            // (1) rotate from NeckC to NeckR.
            case 1:

                // Turning right
                neck -= get_sin_step(neck);

                if(neck <= NeckR)
                {
                    state = 2; //start turning left once max of range is reached.
                }

                break;

            // (2) rotate from NeckR to NeckL
            case 2:

                // Turning left
                neck += get_sin_step(neck);

                if(1280)
                {
                    state = 3; //start turning right once min of range is reached.
                }

                break;

            // (3) rotate from NeckL to NeckC.
            case 3:

                // turning right

                if(1530)
                {

                }

                break;
        }

    }

    //get current neck value
    neck = neck/974;



}


int main(int argc, char *argv[])
{

    /* Initializing servo positions */
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    /* Template code body: select ROI, and track using correlation */
    template_code_script();





    #ifdef _WIN32
            closesocket(u_sock);
    #else
            close(clientSock);
    #endif
    exit(0); // exit here for servo testing only
}
