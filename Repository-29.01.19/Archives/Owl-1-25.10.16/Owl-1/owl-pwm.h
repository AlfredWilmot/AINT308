#ifndef OWLPWM_HPP
#define OWLPWM_HPP

#endif // OWLPWM_HPP

// Defines for servo limits
// PFC Owl robot
// (c) Plymouth University
//
using namespace std;

// OWL eye ranges (max)
int RyBm = 1120; // (bottom) to
int RyTm = 2000; //(top)
int RxRm = 1890; //(right) to
int RxLm = 1200; //(left)
int LyBm = 2000; //(bottom) to
int LyTm = 1180; //(top)
int LxRm = 1850; // (right) to
int LxLm = 1180; // (left)
int NeckR = 1100;
int NeckL = 1950;
// VGA match ranges
int RyBv = 1240; // (bottom) to
int RyTv = 1655; //(top)
int RxRv = 1845; //(right) to
int RxLv = 1245; //(left)
int LyBv = 1880; //(bottom) to
int LyTv = 1420; //(top)
int LxRv = 1835; // (right) to
int LxLv = 1265; // (left)
int RxC=1545;
int RyC=1460;
int LxC=1545;
int LyC=1560;
int NeckC = 1530;
int Ry,Rx,Ly,Lx,Neck; // calculate values for position
//MAX servo eye socket ranges
int RyRangeM=RyTm-RyBm;
int RxRangeM=RxRm-RxLm;
int LyRangeM=LyTm-LyBm; // reflected so negative
int LxRangeM=LxRm-LxLm;
int NeckRange=NeckL-NeckR;
//vga CAMERA ranges
int RyRangeV=RyTv-RyBv;
int RxRangeV=RxRv-RxLv;
int LyRangeV=LyTv-LyBv; // reflected so negative
int LxRangeV=LxRv-LxLv;
