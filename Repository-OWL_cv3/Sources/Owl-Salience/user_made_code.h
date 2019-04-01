#ifndef USER_MADE_CODE_H
#define USER_MADE_CODE_H

#include "owl-pwm.h"

#include "opencv2/calib3d.hpp"

static int Ry,Rx,Ly,Lx,Neck; // calculate values for position

/* variables for storing min/max mat value, after salience model has been applied, to identify most salient point */
static double minVal;
static double maxVal;
static cv::Point minLoc;
static cv::Point maxLoc;
static cv::Point minLocTarget;
static cv::Point maxLocTarget;


void send_servos_home()
{
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;
}

/* Ensure servo doesn't try to go out of bounds */
void servo_boundary_enforcer()
{
    /* enforce Rx boundary */
    if(Rx >= RxRv)
    {
        Rx = RxRv;
    }
    else if(Rx <= RxLv)
    {
        Rx = RxLv;
    }
    else
    {
        Rx = Rx;
    }

    /* enforce Ry boundary */
    if(Ry >= RyTm)
    {
        Ry = RyTm;
    }
    else if(Ry <= RyBm)
    {
        Ry = RyBm;
    }
    else
    {
        Ry = Ry;
    }

    /* enforce Lx boundary */
    if(Lx >= LxRv)
    {
        Lx = LxRv;
    }
    else if(Lx <= LxLv)
    {
        Lx = LxLv;
    }
    else
    {
        Lx = Lx;
    }

    /* enforce Ly boundary */
    if(Ly >= LyBm)
    {
        Ly = LyBm;
    }
    else if(Ly <= LyTm)
    {
        Ly = LyTm;
    }
    else
    {
        Ly = Ly;
    }



}

#endif // USER_MADE_CODE_H
