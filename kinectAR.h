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

#include <amino.h>
#include <ach.h>
#include <syslog.h>

#include <cv.h>

#include <MarkerDetector.h>
#include <GlutViewer.h>

struct tf_qv {
	double rotation[4];
	double translation[3];
};

struct sendMarker
{
	uint8_t counter;
	uint64_t visibleMask;
	struct tf_qv tf[1];
};

class KinectAR 
{
public:
	// constructor
	KinectAR();
	

	void InitGL(int Width, int Height);
	void DispatchDraws();
	void DrawDepthScene();
	void DrawVideoScene();
	void ReSizeGLScene(int Width, int Height);
	
	void video_cb(freenect_device *dev, void *rgb, uint32_t timestamp);
	void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp);
	
	void detectMarkers();
	void openChannel(const char* channelName);
	
	void sendMsg(size_t n);
	void mainLoop();
	
public:
	pthread_t freenect_thread;
	volatile int die;

	int g_argc;
	char **g_argv;

	int depth_window;
	int video_window;

	pthread_mutex_t depth_mutex;
	pthread_mutex_t video_mutex;

	// back: owned by libfreenect (implicit for depth)
	// mid: owned by callbacks, "latest frame ready"
	// front: owned by GL, "currently being drawn"
	uint8_t *depth_mid, *depth_front;
	uint8_t *rgb_back, *rgb_mid, *rgb_front;

	GLuint gl_depth_tex;
	GLuint gl_rgb_tex;

	freenect_context *f_ctx;
	freenect_device *f_dev;
	int freenect_led;

	freenect_video_format requested_format;
	freenect_video_format current_format;
	freenect_resolution requested_resolution;
	freenect_resolution current_resolution;

	pthread_cond_t gl_frame_cond;
	int got_rgb;
	int got_depth;
	int depth_on;
	
	uint16_t t_gamma[2048];
	
	// for AR
	bool init=true;
	double marker_size;
	alvar::Camera cam;
	Drawable d[32];
	alvar::MarkerDetector<alvar::MarkerData> marker_detector;
	
	IplImage* image;
	
	// for ach
	ach_channel_t   channel;
	
};
