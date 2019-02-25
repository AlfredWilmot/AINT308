/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include <iostream>
#include <fstream>


#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>

using namespace cv;
using namespace std;

static void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|sgbm3way] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i=<intrinsic_filename>] [-e=<extrinsic_filename>]\n"
           "[--no-display] [-o=<disparity_image>] [-p=<point_cloud_file>]\n");
}

static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

int main(int argc, char** argv)
{
    std::string Left_filename = "";
    std::string Right_filename = "";
    std::string intrinsic_filename = "../../Data/intrinsics.yml";
    std::string extrinsic_filename = "../../Data/extrinsics.yml";
    std::string disparity_filename = "Disparity.jpg";
    std::string point_cloud_filename = "PointCloud";

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg = STEREO_SGBM; //PFC always do SGBM - colour
    int SADWindowSize, numberOfDisparities;
    bool no_display;
    float scale;

    Ptr<StereoBM> bm = StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    cv::CommandLineParser parser(argc, argv,
                                 "{@arg1||}{@arg2||}{help h||}{algorithm||}{max-disparity|256|}{blocksize|3|}{no-display||}{scale|1|}{i||}{e||}{o||}{p||}");
    if(parser.has("help"))
    {
        print_help();
        return 0;
    }
    //PFC Left_filename = parser.get<std::string>(0);
    //PFC Right_filename = parser.get<std::string>(1);
    if (parser.has("algorithm"))
    {
        std::string _alg = parser.get<std::string>("algorithm");
        alg = _alg == "bm" ? STEREO_BM :
         _alg == "sgbm" ? STEREO_SGBM :
         _alg == "hh" ? STEREO_HH :
         _alg == "var" ? STEREO_VAR :
         _alg == "sgbm3way" ? STEREO_3WAY : -1;
    }
    numberOfDisparities = parser.get<int>("max-disparity");
    SADWindowSize = parser.get<int>("blocksize");
    scale = parser.get<float>("scale");
    no_display = parser.has("no-display");
    if( parser.has("i") )
        intrinsic_filename = parser.get<std::string>("i");
    if( parser.has("e") )
        extrinsic_filename = parser.get<std::string>("e");
    if( parser.has("o") )
        disparity_filename = parser.get<std::string>("o");
    if( parser.has("p") )
        point_cloud_filename = parser.get<std::string>("p");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    if( alg < 0 )
    {
        printf("Command-line parameter error: Unknown stereo algorithm\n\n");
        print_help();
        return -1;
    }
    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
    {
        printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
        print_help();
        return -1;
    }
    if (scale < 0)
    {
        printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
        return -1;
    }
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }
/*PFC    if( Left_filename.empty() || Right_filename.empty() )
    {
        printf("Command-line parameter error: both left and right images must be specified\n");
        return -1;
    }
    */
    if( (!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if( extrinsic_filename.empty() && !point_cloud_filename.empty() )
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

    int color_mode = alg == STEREO_BM ? 0 : -1;
/* PFC
    Mat Left = imread(Left_filename, color_mode);
    Mat Right = imread(Right_filename, color_mode);

    if (Left.empty())
    {
        printf("Command-line parameter error: could not load the first input image file\n");
        return -1;
    }
    if (Right.empty())
    {
        printf("Command-line parameter error: could not load the second input image file\n");
        return -1;
    }

    if (scale != 1.f)
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(Left, temp1, Size(), scale, scale, method);
        Left = temp1;
        resize(Right, temp2, Size(), scale, scale, method);
        Right = temp2;
    }
*/
    Size img_size = {640,480} ; //***PFC BUG fixed was {480,640}; //PFC default to VGA res. always with video feed  was -->//Left.size();

    Rect roi1, roi2;
    Mat Q;

    if( !intrinsic_filename.empty() )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, FileStorage::READ);
        if(!fs.isOpened()){
            printf("Failed to open file %s\n", intrinsic_filename.c_str());
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        M1 *= scale;
        M2 *= scale;

        fs.open(extrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename.c_str());
            return -1;
        }
        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;

        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);


        //VIDEO LOOP PFC March 2017 starts here
        string source ="http://10.0.0.10:8080/stream/video.mjpeg"; // was argv[1];           // the source file name

        VideoCapture cap (source);              // Open input
        if (!cap.isOpened())
        {
            cout  << "Could not open the input video: " << source << endl;
            return -1;
        }
        //Rect region_of_interest = Rect(x, y, w, h);
        bool inLOOP=true;
        cv::Mat Frame,Left,Right;
        cv::Mat disp, disp8;


        while (inLOOP){
            if (!cap.read(Frame))
            {
                cout  << "Could not open the input video: " << source << endl;
                //         break;
            }
            // Mat FrameFlpd; cv::flip(Frame,FrameFlpd,1); // Note that Left/Right are reversed now
            //Mat Gray; cv::cvtColor(Frame, Gray, cv::COLOR_BGR2GRAY);
            // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
            Right= Frame( Rect(0, 0, 640, 480)); // using a rectangle
            Left= Frame( Rect(640, 0, 640, 480)); // using a rectangle
            //DEBUG imshow("Left",Left);imshow("Right", Right);
            //waitKey(30); // display the images



            Mat Leftr, Rightr;
            remap(Left, Leftr, map11, map12, INTER_LINEAR);
            remap(Right, Rightr, map21, map22, INTER_LINEAR);

            Left = Leftr;
            Right = Rightr;


            numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

            bm->setROI1(roi1);
            bm->setROI2(roi2);
            bm->setPreFilterCap(31);
            bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
            bm->setMinDisparity(0);
            bm->setNumDisparities(numberOfDisparities);
            bm->setTextureThreshold(10);
            bm->setUniquenessRatio(15);
            bm->setSpeckleWindowSize(100);
            bm->setSpeckleRange(32);
            bm->setDisp12MaxDiff(1);

            sgbm->setPreFilterCap(63);
            int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
            sgbm->setBlockSize(sgbmWinSize);

            int cn = Left.channels();

            sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
            sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
            sgbm->setMinDisparity(0);
            sgbm->setNumDisparities(numberOfDisparities);
            sgbm->setUniquenessRatio(10);
            sgbm->setSpeckleWindowSize(100);
            sgbm->setSpeckleRange(32);
            sgbm->setDisp12MaxDiff(1);
            if(alg==STEREO_HH)
                sgbm->setMode(StereoSGBM::MODE_HH);
            else if(alg==STEREO_SGBM)
                sgbm->setMode(StereoSGBM::MODE_SGBM);
            else if(alg==STEREO_3WAY)
                sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

            //Mat Leftp, Rightp, dispp;
            //copyMakeBorder(Left, Leftp, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
            //copyMakeBorder(Right, Rightp, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

            int64 t = getTickCount();
            if( alg == STEREO_BM )
                bm->compute(Left, Right, disp);
            else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
                sgbm->compute(Left, Right, disp);
            t = getTickCount() - t;
            printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

            //disp = dispp.colRange(numberOfDisparities, Leftp.cols);
            if( alg != STEREO_VAR )
                disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
            else
                disp.convertTo(disp8, CV_8U);
            if( true )
            {
                //namedWindow("left", 1);
                imshow("left", Left);
                //namedWindow("right", 1);
                imshow("right", Right);
                //namedWindow("disparity", 0);
                imshow("disparity", disp8);
                //printf("press any key to continue...");
                //fflush(stdout);
                char key=waitKey(30);
                if (key=='q') break;
                //printf("\n");
            }
        } // end video loop

        if(!disparity_filename.empty())
            imwrite(disparity_filename, disp8);

        if(!point_cloud_filename.empty())
        {
            printf("storing the point cloud...");
            fflush(stdout);
            Mat xyz;
            reprojectImageTo3D(disp, xyz, Q, true);
            saveXYZ(point_cloud_filename.c_str(), xyz);
            printf("\n");
        }
    } // end got intrinsics IF

    return 0;
}
