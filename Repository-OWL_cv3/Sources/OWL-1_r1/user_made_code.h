#ifndef USER_MADE_CODE_H
#define USER_MADE_CODE_H

#include <opencv2/highgui/highgui.hpp>
#include <iostream>

/* Mouse-click event vars */
static bool _mouse_clk = false;
static int _seed_x = 0;
static int _seed_y = 0;

/*---- Handler method that reacts to user selecting pixel in interactive window ----*/
void mouseEvent(int evt, int x, int y, int, void*)
{

    if (evt == CV_EVENT_LBUTTONDOWN)
    {
        _mouse_clk = true;      //set flag.

        /* Update the new mouse-selected seed pixel coordinates */
        _seed_x = x;
        _seed_y = y;

        std::cout << "Pixel (x,y): " << x << ", " << y << "\n";
    }
}

#endif // USER_MADE_CODE_H
