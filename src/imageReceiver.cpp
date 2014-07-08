/* -*- mode: C++; c-basic-offset: 8; indent-tabs-mode: t;  -*- */
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <time.h>
#include <stdint.h>
#include <ach.h>
#include <sns.h>
#include "imageReceiver.h"

ImageReceiver::ImageReceiver(const char *channelName)
{
	sns_chan_open(&chan, channelName, NULL);
}

Mat ImageReceiver::receiveImage()
{
	size_t fs;
	frame_t *frame;
	ach_status_t r = sns_msg_local_get( &chan, (void**)&frame, &fs,
					    NULL, (ach_get_opts_t)(ACH_O_LAST | ACH_O_WAIT) );

	assert( ACH_OK == r || ACH_MISSED_FRAME == r);

	Mat image(frame->width, frame->height, CV_8UC3);
	int n = frame->width * frame->height;

	// TODO: check than frame size is reasonable

	//assert( n < fs / sizeof(frame->points[0][0]) );
	//std::cout << frame->width << " " << frame->height << std::endl;

	int i = 0;
	for(int x = 0; x < frame->width; x++)
	{
		for(int y = 0; y < frame->height; y++)
		{
			int r = (int)frame->points[i][0];
			int g = (int)frame->points[i][1];
			int b = (int)frame->points[i][2];
			//std::cout << r << " " << g << " " << b << std::endl;
			assert(i < frame->width*frame->height);
			image.at<Vec3b>(x,y)[0] = r;
			image.at<Vec3b>(x,y)[1] = g;
			image.at<Vec3b>(x,y)[2] = b;
			i++;
		}
	}

	return image;
}
