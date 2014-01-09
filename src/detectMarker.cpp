/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "libfreenect.h"

#include <pthread.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <AR/gsub.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/video.h>

#include <kinectAR.h>

pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

KinectAR kinect;


void DispatchDraws() {
	kinect.DispatchDraws();
}

void DrawDepthScene()
{
	kinect.DrawDepthScene();
}

void DrawVideoScene()
{
	kinect.DrawVideoScene();
}

void keyPressed(unsigned char key, int x, int y)
{
}

void ReSizeGLScene(int Width, int Height)
{
	kinect.ReSizeGLScene(Width, Height);
}

void InitGL(int Width, int Height)
{
	kinect.InitGL(Width, Height);
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	kinect.depth_cb(dev, v_depth, timestamp);
}


void* gl_threadfunc(void *arg)
{
	printf("GL thread\n");

	glutInit(&g_argc, g_argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);

	kinect.depth_window = glutCreateWindow("Depth");
	glutDisplayFunc(&DrawDepthScene);
	glutIdleFunc(&DispatchDraws);
	glutKeyboardFunc(&keyPressed);
	InitGL(640, 480);

	freenect_frame_mode mode = freenect_find_video_mode(kinect.current_resolution, kinect.current_format);
	glutInitWindowPosition(640,0);
	glutInitWindowSize(mode.width, mode.height);
	kinect.video_window = glutCreateWindow("Video");

	glutDisplayFunc(&DrawVideoScene);
	glutIdleFunc(&DispatchDraws);
	glutReshapeFunc(&ReSizeGLScene);
	glutKeyboardFunc(&keyPressed);

	InitGL(640, 480);
	ReSizeGLScene(mode.width, mode.height);
	glutMainLoop();

	return NULL;
}

void video_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	kinect.video_cb(dev, rgb, timestamp);
}

void *freenect_threadfunc(void *arg)
{
	//freenect_set_led(f_dev,LED_RED);
	freenect_set_depth_callback(kinect.f_dev, depth_cb);
	freenect_set_video_callback(kinect.f_dev, video_cb);
	
	kinect.mainLoop();
	return NULL;
}


int main(int argc, char **argv)
{
	int res;
	
	// initialize the tracker 
	kinect.openChannel(argv[1]);
	
	res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		freenect_shutdown(kinect.f_ctx);
		return 1;
	}

	// OS X requires GLUT to run on the main thread
	gl_threadfunc(NULL);

	return 0;
}
