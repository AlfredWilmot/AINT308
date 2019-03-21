#ifndef DISTANCE_ESTIMATION_H
#define DISTANCE_ESTIMATION_H

#include "owl-pwm.h"
#include "math.h"

/*----------------------------------*/
/* User made variables & functions */
/*--------------------------------*/

/* Servo calibration values for Owl #1 (measured 08/03/2019) */

//Servo center positions
const int Owl_1_RxC   = 1600;
const int Owl_1_LxC   = 1535;
const int Owl_1_RyC   = 1515;
const int Owl_1_LyC   = 1545;
const int Owl_1_NeckC = 1540;

//Servo center position for owl-08
const int Owl_8_RxC   = 1540;
const int Owl_8_LxC   = 1535;
const int Owl_8_RyC   = 1505;
const int Owl_8_LyC   = 1545;
const int Owl_8_NeckC = 1540;

//Servo minimum positions (TBA)
const int Owl_1_RxMin   = RxLm; //1200
const int Owl_1_LxMin   = LxLm; //1196
const int Owl_1_RyMin   = RyBm;
const int Owl_1_LyMin   = LyTm;
const int Owl_1_NeckMin = NeckR;

//Servo maximum positions (TBA)
const int Owl_1_RxMax  = RxRm; //1890
const int Owl_1_LxMax  = LxRm; //1850
const int Owl_1_RyMax  = RyTm;
const int Owl_1_LyMax  = LyBm;
const int Owl_1_NecMax = NeckL;

const int IPD = 67; //mm


/* Calibration test (08/03/2019) */
const double pi =  3.14159;
const double rad_to_deg = 180/pi;
const double deg_to_rad = pi/180;

// Constant target offset from Owl throughout calibration testing
const double test_target_distance = 400; //mm

// Maximum target lateral displacements at constant offset from Owl.
const double right_eye_max_right_displacement = 320; //mm
const double right_eye_max_left_displacement  = 330; //mm
const double left_eye_max_right_displacement  = 260; //mm
const double left_eye_max_left_displacement   = 340; //mm

// Note: rotation towards fovea is treated as positive.
const double deg_Rx_min = -atan(right_eye_max_right_displacement/ test_target_distance) * rad_to_deg; //-38.66;
const double deg_Rx_max =  atan(right_eye_max_left_displacement/  test_target_distance) * rad_to_deg; //39.52;

const double deg_Lx_min = -atan(left_eye_max_left_displacement/  test_target_distance) * rad_to_deg; //-40.36;
const double deg_Lx_max =  atan(left_eye_max_right_displacement/ test_target_distance) * rad_to_deg; //33.02;

// Number of PWM steps per degree of servo rotation.
const double left_eye_pwm_steps_per_deg  = (Owl_1_LxMax - Owl_1_LxMin) / (deg_Lx_max - deg_Lx_min);
const double right_eye_pwm_steps_per_deg = (Owl_1_RxMax - Owl_1_RxMin) / (deg_Rx_max - deg_Rx_min);

// Calibration error:
// (extrema - mid)*pwm_steps_per_deg = calculated_endpoint_angle
// Take the ratio between this calculated value, and the expected value, to see how well they corroborate.

// Convert PWM step to corresponding angle
static double Rx_theta = 0;
static double Lx_theta = 0;

void update_Rx_theta(){ Rx_theta = (double)(Owl_1_RxC - Rx) / right_eye_pwm_steps_per_deg;}
void update_Lx_theta(){ Lx_theta = -(double)(Owl_1_LxC - Lx) / left_eye_pwm_steps_per_deg; }


// Distance estimation
static double distance_estimate = 0; //mm

void update_distance_estimate()
{
    // if both x-axis angles are positive...
    double estimate_1 = sin((pi/2) - Rx_theta * deg_to_rad) * IPD / (sin(Lx_theta * deg_to_rad));
    double estimate_2 = sin((pi/2) - Lx_theta * deg_to_rad) * IPD / (sin(Rx_theta * deg_to_rad));

    distance_estimate = (estimate_1 + estimate_2) / 2;
}

void update_distance_estimate_PFC()
{
    //Get D_l
    double D_l  = IPD * cos(Rx_theta * deg_to_rad) / sin(Lx_theta * deg_to_rad + Rx_theta * deg_to_rad);
    double A    = D_l;
    double B    = IPD/2;
    double C    = D_l * IPD * sin(Lx_theta * deg_to_rad);

    distance_estimate = sqrt(A*A + B*B - C);
}

void stereo_eyes()
{
 //fix the eyes in a position for stereo calibrations. miniscule toe in

}

#endif // DISTANCE_ESTIMATION_H
