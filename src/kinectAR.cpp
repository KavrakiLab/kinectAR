/* -*- mode: C++; c-basic-offset: 8; indent-tabs-mode: t;  -*- */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#include "kinectAR.h"
#include <sns.h>

const double fudgeParam = 0.175;

static inline size_t msg_size( size_t n ) {
	return sizeof(struct sendMarker) - sizeof(struct tf_qv) + (n) * sizeof(struct tf_qv);
}

uint64_t  mask_set_i(uint64_t mask, uint8_t i, int is_visible)
{
	return is_visible ? mask | (1<<i) : mask;

}

#define ALLOCA_MSG(n) ( (struct sendMarker*)alloca( msg_size(n) ) )

KinectAR::KinectAR()
{
	// color shading
	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

	pthread_mutex_t depth_mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_mutex_t video_mutex = PTHREAD_MUTEX_INITIALIZER;

	requested_format = FREENECT_VIDEO_RGB;
	current_format = FREENECT_VIDEO_RGB;
	requested_resolution = FREENECT_RESOLUTION_HIGH;
	current_resolution = FREENECT_RESOLUTION_HIGH;

	gl_frame_cond = PTHREAD_COND_INITIALIZER;
	got_rgb = 0;
	got_depth = 0;
	depth_on = 1;

	depth_mid = (uint8_t*)malloc(640*480*3);
	depth_front = (uint8_t*)malloc(640*480*3);

	printf("Kinect camera test\n");

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		exit(0);
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	int user_device_number = 0;

	if (nr_devices < 1) {
		freenect_shutdown(f_ctx);
		exit(0);
	}

	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		exit(0);
	}

	marker_size = 2.8 - fudgeParam;
	image = cvCreateImage(cvSize(1280,1024), IPL_DEPTH_8U, 3);
}

void KinectAR::detectMarkers()
{
	static int a = 0;
	//std::cout << "Frame #: " << a << std::endl;
	a++;

	image->imageData = (char *)rgb_mid;

	if (init)
	{
		init = false;
		//cout<<"Loading calibration: "<<calibrationFilename.str();
		if (false)
		{
			std::cout<<" [Ok]"<< std::endl;
		}
		else
		{
			cam.SetRes(image->width, image->height);
			std::cout<<" [Fail]"<< std::endl;
		}

		double p[16];
		cam.GetOpenglProjectionMatrix(p,image->width,image->height);
		GlutViewer::SetGlProjectionMatrix(p);
		for (int i=0; i<32; i++)
		{
			d[i].SetScale(marker_size);
		}
	}

	marker_detector.SetMarkerSize(marker_size); // for marker ids larger than 255, set the content resolution accordingly
	marker_detector.Detect(image, &cam, true, true);

	GlutViewer::DrawableClear();
	for (size_t i=0; i<marker_detector.markers->size(); i++)
	{
		if (i >= 32) break;
		alvar::Pose p = (*(marker_detector.markers))[i].pose;
		p.GetMatrixGL(d[i].gl_mat);

		int id = (*(marker_detector.markers))[i].GetId();
		double r = 1.0 - double(id+1)/32.0;
		double g = 1.0 - double(id*3%32+1)/32.0;
		double b = 1.0 - double(id*7%32+1)/32.0;
		d[i].SetColor(r, g, b);

		GlutViewer::DrawableAdd(&(d[i]));
		//std::cout << p.translation[0] << " " << p.translation[1] << " " << p.translation[2] << std::endl;
	}
}

void KinectAR::DispatchDraws()
{
	pthread_mutex_lock(&depth_mutex);
	if (got_depth) {
		glutSetWindow(depth_window);
		glutPostRedisplay();
	}
	pthread_mutex_unlock(&depth_mutex);
	pthread_mutex_lock(&video_mutex);
	if (got_rgb) {
		glutSetWindow(video_window);
		glutPostRedisplay();
	}
	pthread_mutex_unlock(&video_mutex);
}

void KinectAR::DrawDepthScene()
{
	pthread_mutex_lock(&depth_mutex);
	if (got_depth) {
		uint8_t* tmp = depth_front;
		depth_front = depth_mid;
		depth_mid = tmp;
		got_depth = 0;
	}
	pthread_mutex_unlock(&depth_mutex);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_front);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0, 0); glVertex3f(0,0,0);
	glTexCoord2f(1, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 1); glVertex3f(640,480,0);
	glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();

	detectMarkers();
	// send the data
	sendMsg(32);


	glutSwapBuffers();
}

void KinectAR::DrawVideoScene()
{
	if (requested_format != current_format || requested_resolution != current_resolution) {
		return;
	}

	pthread_mutex_lock(&video_mutex);

	freenect_frame_mode frame_mode = freenect_get_current_video_mode(f_dev);

	if (got_rgb) {
		uint8_t *tmp = rgb_front;
		rgb_front = rgb_mid;
		rgb_mid = tmp;
		got_rgb = 0;
	}

	pthread_mutex_unlock(&video_mutex);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	if (current_format == FREENECT_VIDEO_RGB || current_format == FREENECT_VIDEO_YUV_RGB) {
		glTexImage2D(GL_TEXTURE_2D, 0, 3, frame_mode.width, frame_mode.height, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb_front);
	} else {
		glTexImage2D(GL_TEXTURE_2D, 0, 1, frame_mode.width, frame_mode.height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, rgb_front);
	}
	glBegin(GL_TRIANGLE_FAN);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0, 0); glVertex3f(0,0,0);
	glTexCoord2f(1, 0); glVertex3f(frame_mode.width,0,0);
	glTexCoord2f(1, 1); glVertex3f(frame_mode.width,frame_mode.height,0);
	glTexCoord2f(0, 1); glVertex3f(0,frame_mode.height,0);
	glEnd();

	glutSwapBuffers();
	//glutPostRedisplay();
}

void KinectAR::ReSizeGLScene(int Width, int Height)
{
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, Width, Height, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

void KinectAR::InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glGenTextures(1, &gl_rgb_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	ReSizeGLScene(Width, Height);
}


void KinectAR::video_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	pthread_mutex_lock(&video_mutex);

	// swap buffers
	assert (rgb_back == rgb);
	rgb_back = rgb_mid;
	freenect_set_video_buffer(dev, rgb_back);
	rgb_mid = (uint8_t*)rgb;

	got_rgb++;
	pthread_mutex_unlock(&video_mutex);
}

void KinectAR::mainLoop()
{
	freenect_set_video_mode(f_dev, freenect_find_video_mode(current_resolution, current_format));
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));

	rgb_back  = (uint8_t*)malloc(freenect_find_video_mode(current_resolution, current_format).bytes);
	rgb_mid   = (uint8_t*)malloc(freenect_find_video_mode(current_resolution, current_format).bytes);
	rgb_front = (uint8_t*)malloc(freenect_find_video_mode(current_resolution, current_format).bytes);

	freenect_set_video_buffer(f_dev, rgb_back);
	freenect_start_depth(f_dev);
	freenect_start_video(f_dev);

	int status = 0;
	while (!sns_cx.shutdown && status >= 0) {
		aa_mem_region_local_release();
		status = freenect_process_events(f_ctx);
		if (requested_format != current_format || requested_resolution != current_resolution) {
			freenect_stop_video(f_dev);
			freenect_set_video_mode(f_dev, freenect_find_video_mode(requested_resolution, requested_format));
			pthread_mutex_lock(&video_mutex);
			free(rgb_back);
			free(rgb_mid);
			free(rgb_front);
			rgb_back = (uint8_t*)malloc(freenect_find_video_mode(requested_resolution, requested_format).bytes);
			rgb_mid = (uint8_t*)malloc(freenect_find_video_mode(requested_resolution, requested_format).bytes);
			rgb_front = (uint8_t*)malloc(freenect_find_video_mode(requested_resolution, requested_format).bytes);
			current_format = requested_format;
			current_resolution = requested_resolution;
			pthread_mutex_unlock(&video_mutex);
			freenect_set_video_buffer(f_dev, rgb_back);
			freenect_start_video(f_dev);
		}
	}

	if (status < 0) {
		printf("Something went terribly wrong.  Aborting.\n");
		exit(0);
	}

	printf("\nshutting down streams...\n");

	freenect_stop_depth(f_dev);
	freenect_stop_video(f_dev);

	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	printf("-- done!\n");
}

void KinectAR::depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	int i;
	uint16_t *depth = (uint16_t*)v_depth;

	pthread_mutex_lock(&depth_mutex);
	for (i=0; i<640*480; i++) {
		int pval = t_gamma[depth[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
			case 0:
				depth_mid[3*i+0] = 255;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+2] = 255-lb;
				break;
			case 1:
				depth_mid[3*i+0] = 255;
				depth_mid[3*i+1] = lb;
				depth_mid[3*i+2] = 0;
				break;
			case 2:
				depth_mid[3*i+0] = 255-lb;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+2] = 0;
				break;
			case 3:
				depth_mid[3*i+0] = 0;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+2] = lb;
				break;
			case 4:
				depth_mid[3*i+0] = 0;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+2] = 255;
				break;
			case 5:
				depth_mid[3*i+0] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+2] = 255-lb;
				break;
			default:
				depth_mid[3*i+0] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+2] = 0;
				break;
		}
	}
	got_depth++;
	pthread_mutex_unlock(&depth_mutex);
}


void KinectAR::openChannel(const char* channelName)
{
	sns_chan_open( &channel, channelName, NULL );
}

void KinectAR::sendMsg(size_t n)
{
	// set the number of objects
	struct timespec now = sns_now();

	sns_msg_wt_tf *msg = sns_msg_wt_tf_local_alloc(n);

	// loop over all markers
	for (size_t i=0; i<marker_detector.markers->size(); i++)
	{
		int id = (*(marker_detector.markers))[i].GetId();
		sns_wt_tf *wt_tf = &msg->wt_tf[id];

		alvar::Pose p = (*(marker_detector.markers))[i].pose;

		// get the quaternion orientation
		double tmp[4];
		CvMat mat = cvMat(4, 1, CV_64F, tmp);
		p.GetQuaternion(&mat);
		double* test = (double*)mat.data.ptr;

		// set visibility		
		wt_tf->weight = 1;

		// set position
		// set the positions
		for(int j = 0; j < 3; j++)
			wt_tf->tf.v.data[j] = p.translation[j] * 1e-2;

		// set the orientation
		wt_tf->tf.r.w = *(test+0);
		wt_tf->tf.r.x = *(test+1);
		wt_tf->tf.r.y = *(test+2);
		wt_tf->tf.r.z = *(test+3);
	}

	// send out the message via ACH
	// put it into the channel
	if(marker_detector.markers->size() > 0)
	{
		enum ach_status r = sns_msg_wt_tf_put( &channel, msg );
		if( ACH_OK != r )
		{
			syslog( LOG_ERR, "Could not put data: %s\n", ach_result_to_string(r) );
		}

		// some debug messages
		for(int i = 0; i < marker_detector.markers->size(); i++)
		{
			int currId = (*(marker_detector.markers))[i].GetId();
			std::cout << "[ROT]: "
				  << msg->wt_tf[currId].tf.r.x << " "
				  << msg->wt_tf[currId].tf.r.y << " "
				  << msg->wt_tf[currId].tf.r.z << " "
				  << msg->wt_tf[currId].tf.r.w << " "
				  << std::endl;
			std::cout << "[POS]: "
				  << msg->wt_tf[currId].tf.v.x << " "
				  << msg->wt_tf[currId].tf.v.y << " "
				  << msg->wt_tf[currId].tf.v.z << " "
				  << std::endl;
		}
	}
}
