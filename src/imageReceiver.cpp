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

Mat *ImageReceiver::receiveImage(Mat *m)
{
	size_t fs;
	frame_t *frame;
	ach_status_t r = sns_msg_local_get( &chan, (void**)&frame, &fs,
					    NULL, (ach_get_opts_t)(ACH_O_LAST | ACH_O_WAIT) );
	switch( r )  {
	case ACH_CANCELED:
		return NULL;
	case ACH_OK:
	case ACH_MISSED_FRAME:
		break;
	default:
		SNS_DIE( "Could not receive image frame: %s\n",
			 ach_result_to_string(r) );
	}

	if( m ) {
		SNS_REQUIRE( m->rows == frame->width &&
			     m->cols == frame->height,
			     "Size mismatch on receiving frame as matrix\n" );
	} else  {
		m = new Mat(frame->width, frame->height, CV_8UC3);
	}
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
			m->at<Vec3b>(x,y)[0] = r;
			m->at<Vec3b>(x,y)[1] = g;
			m->at<Vec3b>(x,y)[2] = b;
			i++;
		}
	}

	return m;
}

Mat *ImageReceiver::receiveImage2(Mat *m)
{
	size_t fs;
	frame_t *frame;
	ach_status_t r = sns_msg_local_get( &chan, (void**)&frame, &fs,
					    NULL, (ach_get_opts_t)(ACH_O_LAST | ACH_O_WAIT) );
	switch( r )  {
	case ACH_CANCELED:
		return NULL;
	case ACH_OK:
	case ACH_MISSED_FRAME:
		break;
	default:
		SNS_DIE( "Could not receive image frame: %s\n",
			 ach_result_to_string(r) );
	}

	if( m ) {
		SNS_REQUIRE( m->rows == frame->height &&
			     m->cols == frame->width,
			     "Size mismatch on receiving frame as matrix\n" );
	} else  {

		m = new Mat(frame->height, frame->width, CV_8UC3, Scalar(0,0,255));
		//m = new Mat(frame->width, frame->height, CV_8UC3);
	}
	// TODO: check than frame size is reasonable

	int k = 0;
	for(int i = 0; i < frame->height; i++)
	{
		for(int j = 0; j < frame->width; j++)
		{
			int r = (int)frame->points[k][0];
			int g = (int)frame->points[k][1];
			int b = (int)frame->points[k][2];

			assert(i < frame->width*frame->height);
			m->at<Vec3b>(i,j)[0] = r;
			m->at<Vec3b>(i,j)[1] = g;
			m->at<Vec3b>(i,j)[2] = b;
			k++;
		}
	}
	return m;
}
