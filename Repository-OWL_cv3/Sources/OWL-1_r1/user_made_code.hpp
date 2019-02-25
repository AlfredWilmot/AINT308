#ifndef USER_MADE_CODE_HPP_
#define USER_MADE_CODE_HPP_

#include <opencv2/highgui/highgui.hpp>
#include <iostream>

/* Mouse-click event vars */
static bool _mouse_clk = false;
static int _seed_x = 0;
static int _seed_y = 0;

/*---- Handler method that reacts to user selecting pixel in interactive window ----*/
extern void mouseEvent(int evt, int x, int y, int, void*);


#endif // USER_MADE_CODE_H
