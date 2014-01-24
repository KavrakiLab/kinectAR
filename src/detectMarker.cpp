/* -*- mode: C++; c-basic-offset: 8; indent-tabs-mode: t;  -*- */
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
 *	accompanying it with the APACHE20 and GPL20 files, or
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

#include <sns.h>

using namespace cv;
using namespace std;


KinectAR kinect;

int main(int argc, char **argv)
{
	// draw image
	while(true)
	{
		if( !kinect.capture.grab() )
		{
			cout << "Can not grab images." << endl;
			return -1;
		}
		else
		{
			kinect.DrawScene();
			kinect.DetectMarkers();
			kinect.CreatePointCloud();
			//kinect.SendMsg(32);
		}
		
		if( waitKey( 30 ) >= 0 )
			kinect.Keyboard('n', 1, 1);
		//break;
	}

return 0;
}
