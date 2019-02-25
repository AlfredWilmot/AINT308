#include "user_made_code.h"


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
