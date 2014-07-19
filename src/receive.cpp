/* -*- mode: C++; c-basic-offset: 8; indent-tabs-mode: t;  -*- */
#include <amino.h>
#include <ach.h>
#include <sns.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>
#include <sns.h>

#include "imageReceiver.h"

using namespace cv;
using namespace std;

int main( int argc, char* argv[] )
{
	sns_init();
	if(argc < 2)
	{
		std::cout << "Usage: ./receive channelname" << std::endl;
		return 0;
	}

	ImageReceiver rec(argv[1]);

	sns_start();
	while(!sns_cx.shutdown)
	{
		Mat *m = rec.receiveImage2(NULL);
		if(m) {
			imshow( "receive", *m );
			delete m;
		}

		waitKey( 30 );

		aa_mem_region_local_release();
	}
	return 0;
}
