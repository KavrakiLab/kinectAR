/* -*- mode: C++; c-basic-offset: 8; indent-tabs-mode: t;  -*- */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Heni Ben Amor
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <amino.h>
#include <unistd.h>
#include <ach.h>
#include <sns.h>

#include <iostream>

using namespace cv;
using namespace std;

ach_channel_t chan;

int width = 1920;
int height = 1080;
char* channelName;
bool kinect = false;
int cameraID = 0;

typedef struct {
	size_t width;
	size_t height;
	char points[1][3];
} frame_t;

int size;
frame_t* frame;

/*
 * Open ach channel
 */
void openChannel()
{
	// calculate size of frame
	size = sizeof(frame) + sizeof(frame->points[0])*(width*height*3);
	frame = (frame_t*) (malloc(size));

	// print out the size of the frame
	std::cout << "Frame size: " << size << std::endl;

	// open ach
	sns_chan_open( &chan, channelName, NULL );
}

/*
 * Sends an image over ach
 */
void sendImage(const Mat &image, ach_channel_t* ch)
{
	frame->width = image.cols;
	frame->height = image.rows;
	int i = 0;

	for(int x = 0; x < frame->height; x++)
	{
		for(int y = 0; y < frame->width; y++)
		{

			char *rgb = (frame)->points[(i)];
			rgb[0] = (char) image.at<cv::Vec3b>(x,y)[0];
			rgb[1] = (char) image.at<cv::Vec3b>(x,y)[1];
			rgb[2] = (char) image.at<cv::Vec3b>(x,y)[2];
			assert(i < frame->width*frame->height);
			i++;
		}
	}
	SNS_LOG(LOG_DEBUG, "image_size: %lu %lu %lu\n", frame->width, frame->height, size );
	ach_status_t r = ach_put( &chan, frame, size );
	SNS_REQUIRE( ACH_OK == r, "Could not put frame: %s\n", ach_result_to_string(r) );
}

/*
 * This program reads images from a camera and distributes them via the
 * ACH IPC protocol
 */
int main( int argc, char* argv[] )
{
	int c;
	int imageMode;
	string filename;
	bool isVideoReading;

	// init sns
	sns_init();

	// parse command line arguments
	while((c = getopt(argc, argv, "w:h:c:n:k?")) != -1)
	{
		switch(c)
		{
			SNS_OPTCASES;
			case 'w':
				width = atoi(optarg);
				break;
			case 'h':
				height = atoi(optarg);
				break;
			case 'c':
				channelName = optarg;
				break;
			case 'n':
				cameraID = atoi(optarg);
				break;
			case 'k':
				kinect = true;
				std::cout << "->USING KINECT! " <<  channelName << std::endl;
				break;
			case '?':
			default:
				puts( "Usage: streampic -w resx -h resy -c channel -k\n"
				      "Detect markers with kinect\n"
				      "\n"
				      "Options:\n"
				      "  -w resx,                  Set the x resolution (1920)\n"
				      "  -h resy,                  Set the y resolution (1080) \n"
				      "  -n cameraID,              Set id of camera\n"
				      "  -c CHANNEL,               Set output Ach channel\n"
				      "  -k kinect,                Use kinect\n"
				      "  -?,                       Give program help list\n"
				      "\n"
				      "Examples:\n"
				      "streampic -c video1 -w 1920 -h 1080 -n 2\n"
				      ""
				      "Report bugs to <hbenamor@cc.gatech.edu>" );
				return 0;
		}
	}

	// open camera
	cout << "Device opening ..." << endl;
	VideoCapture capture;
	if(kinect)
		capture.open( CV_CAP_OPENNI );
	else
		capture.open( cameraID );
	cout << "done." << endl;

	if( !capture.isOpened() )
	{
		cout << "Can not open a capture object." << endl;
		return -1;
	}

	// set kinect high res mode
	if( kinect )
	{
		width = 1280;
		height = 1024;
		bool modeRes=false;

		modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_SXGA_15HZ );
	}
	else // set resolution
	{
		capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	}

	// open ach channel
	openChannel();

	sns_start();
	while(!sns_cx.shutdown)
	{
		Mat depthMap;
		Mat bgrImage;

		if(kinect)
		{
			if( !capture.grab() )
			{
				cout << "Can not grab images." << endl;
				return -1;
			}
			else
			{
				capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE );
				capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
			}
		}
		else
		{
			capture >> bgrImage;
		}
		sendImage(bgrImage, &chan);
	}

	aa_mem_region_local_release();
	free(frame);
	return 0;
}
