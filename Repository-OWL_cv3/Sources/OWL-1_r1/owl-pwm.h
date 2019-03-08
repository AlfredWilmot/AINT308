#ifndef OWLPWM_HPP
#define OWLPWM_HPP

#endif // OWLPWM_HPP

// Defines for servo limits
// PFC Owl robot
// (c) Plymouth University

#include <math.h>

// OWL eye ranges (max)
static int RyBm = 1120; // (bottom) to
static int RyTm = 2000; //(top)
static int RxRm = 1890; //(right) to
static int RxLm = 1200; //(left)
static int LyBm = 2000; //(bottom) to
static int LyTm = 1180; //(top)
static int LxRm = 1850; // (right) to
static int LxLm = 1180; // (left)
static int NeckR = 1100;
static int NeckL = 1950;
// VGA match ranges
static int RyBv = 1240; // (bottom) to
static int RyTv = 1655; //(top)
static int RxRv = 1845; //(right) to
static int RxLv = 1245; //(left)
static int LyBv = 1880; //(bottom) to
static int LyTv = 1420; //(top)
static int LxRv = 1835; // (right) to
static int LxLv = 1265; // (left)
static int RxC=1445;//1545;
static int RyC=1390;//1460;
static int LxC=1470;//1545;
static int LyC=1595;//560;
static int NeckC = 1540;
static int Ry,Rx,Ly,Lx,Neck; // calculate values for position
//MAX servo eye socket ranges
static int RyRangeM=RyTm-RyBm;
static int RxRangeM=RxRm-RxLm;
static int LyRangeM=LyTm-LyBm; // reflected so negative
static int LxRangeM=LxRm-LxLm;
static int NeckRange=NeckL-NeckR;
//vga CAMERA ranges
static int RyRangeV=RyTv-RyBv;
static int RxRangeV=RxRv-RxLv;
static int LyRangeV=LyTv-LyBv; // reflected so negative
static int LxRangeV=LxRv-LxLv;

/*----------------------------------*/
/* User made variables & functions */
/*--------------------------------*/

/* Servo calibration values for Owl #1 (measured 08/03/2019) */

//Servo center positions
const int Owl_1_RxC   = 1590;
const int Owl_1_LxC   = 1535;
const int Owl_1_RyC   = 1515;
const int Owl_1_LyC   = 1545;
const int Owl_1_NeckC = 1540;

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
const double deg_to_rad = pi/180;

// Constant target offset from Owl throughout calibration testing
const double test_target_distance = 400; //mm

// Maximum target lateral displacements at constant offset from Owl.
const double right_eye_max_right_displacement = 320; //mm
const double right_eye_max_left_displacement  = 330; //mm
const double left_eye_max_right_displacement  = 260; //mm
const double left_eye_max_left_displacement   = 340; //mm

// Note: rotation towards fovea is treated as positive.
const double deg_Rx_min = -atan(right_eye_max_right_displacement/ test_target_distance); //-38.66;
const double deg_Rx_max =  atan(right_eye_max_left_displacement/  test_target_distance); //39.52;

const double deg_Lx_min = -atan(left_eye_max_left_displacement/  test_target_distance); //-40.36;
const double deg_Lx_max =  atan(left_eye_max_right_displacement/ test_target_distance); //33.02;

// Number of PWM steps per degree of servo rotation.
const double left_eye_pwm_steps_per_deg  = (Owl_1_LxMax - Owl_1_LxMin) / (deg_Lx_max - deg_Lx_min);
const double right_eye_pwm_steps_per_deg = (Owl_1_RxMax - Owl_1_RxMin) / (deg_Rx_max - deg_Rx_min);

// Convert PWM step to corresponding angle

