#include "template_code.h"

#include <math.h>
#include <time.h>
#include <string>
using namespace std;
using namespace cv;


/* Moves all servos to the position corresponding to the latest PWM mark-period value */
void update_servo_position()
{
    /* Construct servo control packet */
    CMDstream.str("");
    CMDstream.clear();
    CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
    CMD = CMDstream.str();
    string RxPacket= OwlSendPacket (u_sock, CMD.c_str());

    /* Induce delay to give packet enought time to send */
    waitKey(20);
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
static int step_Rx   = 0;
static int step_Ry   = 0;
static int step_Lx   = 0;
static int step_Ly   = 0;
static int step_Neck = 0;

#define pi 3.14159265358979323846

/* input position start & stop, and the axis to be actuated */
int sine_motion(int start, int stop, std::string axis_to_move)
{
    double midpoint = abs(start - stop)/2.0;

    double scaling_factor = pi/(start+stop);



    if(axis_to_move == "Rx")
    {
        if(step_Rx == 0)
        {
            step_Rx = start;
        }
        else
        {
             step_Rx += sin(step_Rx/scaling_factor);
        }


    }
    else if(axis_to_move == "Ry")
    {

    }
    else if(axis_to_move == "Lx")
    {

    }
    else if(axis_to_move == "Ly")
    {

    }
    else if(axis_to_move == "Neck")
    {

    }

    double step = sin(scaling_factor);

    /* Debuggery */
    cout << "user passed: " << axis_to_move;

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


int main(int argc, char *argv[])
{

    /* Initializing servo positions */
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    /* Template code body: select ROI, and track using correlation */
    //template_code_script();

    while(1)
    {
        //sinusoidal_neck();
        sine_motion(1100, 1950, "Neck");
        waitKey(100);
    }


    #ifdef _WIN32
            closesocket(u_sock);
    #else
            close(clientSock);
    #endif
    exit(0); // exit here for servo testing only
}
