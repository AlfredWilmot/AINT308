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

    cout << "\nRx:" << Rx << "\nRy:" << Ry << "\nLx:" << Lx << "\nLy:" << Ly << "\nNeck:"  << Neck << "\n";

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
static int step_Rx   = -1;
static int step_Ry   = -1;
static int step_Lx   = -1;
static int step_Ly   = -1;
static int step_Neck = -1;

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
}

int main(int argc, char *argv[])
{

    /* Initializing servo positions */
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    /* Template code body: select ROI, and track using correlation */
    //template_code_script();

    int stage = 0;


    /* flags used to keep track of progress of each axis during actuation (allows for pseudo-concurrent movement of axes)*/
    int Rx_done    = 0;
    int Ry_done    = 0;
    int Lx_done    = 0;
    int Ly_done    = 0;
    int Neck_done  = 0;






    while(1)
    {






    }


    #ifdef _WIN32
            closesocket(u_sock);
    #else
            close(clientSock);
    #endif
    exit(0); // exit here for servo testing only
}


