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
	ImageReceiver() {initialized = false;}
	void init(const char* channelName, int resX, int resY );
	Mat receiveImage();

private:
	int size, iHeight, iWidth;
	ach_channel_t chan;
	frame_t* frame;
	bool initialized;
};
