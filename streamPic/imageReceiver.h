#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <amino.h>
#include <ach.h>
#include <sns.h>

#include <iostream>

using namespace cv;
using namespace std;

typedef struct {
    size_t width;
    size_t height;
    char points[1][3];
} frame_t;

class ImageReceiver
{
public:
	ImageReceiver( char* channelName, int resX, int resY )
	{
	    int r = ach_open(&chan, channelName, NULL);
	    assert( ACH_OK == r );

	    iWidth  = resX;
	    iHeight = resY;

	    size = sizeof(frame) + sizeof(frame->points[0])*(iWidth*iHeight*3);
	    frame = (frame_t*) (malloc(size));
	}

	Mat receiveImage()
	{
		size_t fs;
		int r = ach_get( &chan, frame, size, &fs,
		             NULL, (ach_get_opts_t)(ACH_O_LAST | ACH_O_WAIT) );

		assert( ACH_OK == r || ACH_MISSED_FRAME == r );

		int n = frame->width * frame->height;

		//assert( n < fs / sizeof(frame->points[0][0]) );
		Mat image(frame->height, frame->width, CV_8UC3, Scalar(0,0,255));
		int k = 0;
		for(int i = 0; i < frame->height; i++)
		{
			for(int j = 0; j < frame->width; j++)
			{
		    		int r = (int)frame->points[k][0];
		    		int g = (int)frame->points[k][1];
		    		int b = (int)frame->points[k][2];

		    		assert(i < frame->width*frame->height);
				image.at<Vec3b>(i,j)[0] = r;
				image.at<Vec3b>(i,j)[1] = g;
				image.at<Vec3b>(i,j)[2] = b;
				k++;
			}
		}

		return image;
		aa_mem_region_local_release();
	}

private:
	int size, iHeight, iWidth;
	ach_channel_t chan;
	frame_t* frame;

};






