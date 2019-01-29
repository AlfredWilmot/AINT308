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

#include <math.h>
#include <time.h>

using namespace std;
using namespace cv;


/* Moves all servos to the position corresponding to the latest PWM mark-period value */
void update_servo_position()
{
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());
}

/* Defines the step that should be taken as a sinusoidal function of the current neck position */
int get_sin_step(int neck_val)
{
 int step = int( round( (sin(neck_val/974) - 0.904)*100 ) ) ;

 return step;

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
                //neck -= get_sin_step(neck);
                Neck -= 5;

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
                //neck += get_sin_step(neck);
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
                // -=get_sin_step(neck);
                Neck -= 5;

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
        else
        {
            waitKey(20);
        }
    }
}


int main(int argc, char *argv[])
{

    /* Initializing servo positions */
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    /* Template code body: select ROI, and track using correlation */
    //template_code_script();

    sinusoidal_neck();



    #ifdef _WIN32
            closesocket(u_sock);
    #else
            close(clientSock);
    #endif
    exit(0); // exit here for servo testing only
}
