#include "../include/imageReceiver.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>

using namespace cv;
using namespace std;

int main( int argc, char* argv[] )
{
    sns_init();
    if(argc < 2)
    {
	std::cout << "Usage: ./receive channelname image_width image_height" << std::endl;
	return 0;
    }

    ImageReceiver rec(argv[1], atoi(argv[2]), atoi(argv[3]));

    std::cout << "Main loop" << std::endl;
    while(!sns_cx.shutdown)
    {
        Mat image;

	// send image over ach
	image = rec.receiveImage();

	imshow( "receive", image );

        if( waitKey( 30 ) >= 0 )
            break;

	aa_mem_region_local_release();
    }
    return 0;
}
