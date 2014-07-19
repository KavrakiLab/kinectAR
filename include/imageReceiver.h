/* -*- mode: C++; c-basic-offset: 8; indent-tabs-mode: t;  -*- */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cstdio>
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
	ImageReceiver(const char *channelName);
	Mat *receiveImage(Mat *m);
	Mat *receiveImage2(Mat *m);
	ach_channel_t chan;
};
