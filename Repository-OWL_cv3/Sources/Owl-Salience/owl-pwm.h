#ifndef OWLPWM_HPP
#define OWLPWM_HPP

#endif // OWLPWM_HPP

// Defines for servo limits
// PFC Owl robot
// (c) Plymouth University


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
//static int Ry,Rx,Ly,Lx,Neck; // calculate values for position
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
