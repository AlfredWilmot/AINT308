#include "template_code.h"

#include <math.h>
#include <time.h>
#include <string>
using namespace std;
using namespace cv;


int main(int argc, char *argv[])
{



    /* Template code body: select ROI, and track using correlation */
    template_code_script();


    #ifdef _WIN32
            closesocket(u_sock);
    #else
            close(clientSock);
    #endif
    exit(0); // exit here for servo testing only
}


