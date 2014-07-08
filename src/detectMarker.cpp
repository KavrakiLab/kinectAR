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

#include <kinectAR.h>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <getopt.h>

#include <osgGA/TrackballManipulator>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>

#include <iostream>
#include <sns.h>

#include "cparams.h"

using namespace cv;
using namespace std;

CamMode mode;
bool useKinect   = false;
bool useGraphics = false;

const char *opt_chan_tf = "markers";
const char *opt_chan_cam = "video";
char *param_file        = NULL;
char *inp_type          = "normal";

/**
 * Creates a scene graph for 3D visualization
 */
osg::Group* createSceneGraph(osgViewer::Viewer* viewer)
{
	// create the root node of the scenegraph
	osg::Group *root = new osg::Group;

	// add the table
	osg::Geode* table	 = new osg::Geode;
	osg::ShapeDrawable* shape  = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),80.0f,40.0f,0.05f));
	osg::PositionAttitudeTransform* transf = new osg::PositionAttitudeTransform;

	transf->setPosition(osg::Vec3(0,0,125));
	root->addChild(transf);
	transf->addChild(table);
	table->addDrawable(shape);

	// add a viewport to the viewer and attach the scene graph.
	viewer->setSceneData(root);

	// set the trackball manipulator
	viewer->setCameraManipulator(new osgGA::TrackballManipulator());

	// only initialize Scenegraph if we actually want to display something
	viewer->realize();

	return root;
}

/**
 * Main loop
 */
int main(int argc, char **argv)
{
	sns_init();
	for( int c; -1 != (c = getopt(argc, argv, "t:p:gi:o:x?" SNS_OPTSTRING)); ) {
		switch(c) {
			SNS_OPTCASES;
		case 't':
			// analyze args
			if(optarg[0] == 'a')
				mode = ACH;
			else if (optarg[0] == 'k')
				mode = KINECT;
			else
				mode = NORMAL;
			break;
		case 'i':
			opt_chan_cam = optarg;
			break;
		case 'o':
			opt_chan_tf = optarg;
			break;
		case 'p':
			param_file = optarg;
			break;
		case 'x':
			useGraphics = true;
			break;
		case '?':   /* help     */
		default:
			puts( "Usage: detect_marker -c channel -i (normal/kinect/ach)\n"
			"Detect markers with kinect\n"
			"\n"
			"Options:\n"
			"  -i CHANNEL,                  Input Ach channel (camera)\n"
			"  -o CHANNEL,                  Output Ach channel (markers tfs)\n"
			"  -t INPUT_TYPE,               Set type of video input: ACH, NORMAL, KINECT\n"
			"  -p file,                     Set the parameter file from which to read additional params\n"
			"  -?,                          Give program help list\n"
			"\n"
			"Report bugs to <hbenamor@cc.gatech.edu>" );
			break;
		}
	}
	// initialize parameters
	CParams p;

	// load from file
	if(param_file != NULL)
	{
		p = CParams(param_file);
	}

	// create a camera processing module
	KinectAR camera("calib.xml", p, opt_chan_cam, opt_chan_tf);

	// create a scene graph
	// use an ArgumentParser object to manage the program arguments.
	int numArgs = 6;
	char* args[numArgs];
	args[0]= "program";
	args[1]= "--window";
	args[2]= "100";
	args[3]= "100";
	args[4]= "400";
	args[5]= "400";

	osg::ArgumentParser arguments(&numArgs, args);
	osgViewer::Viewer* viewer;
	osg::Group* root;

	// add to the sceneGraph
	// only add to scene graph if we are visualizing things
	if(useGraphics)
	{
		viewer = new osgViewer::Viewer(arguments);

		root = createSceneGraph(viewer);
		for(int i = 0; i < 32; i++)
		{
			(camera.kinectMarkers[i]).AddToSceneGraph(root);
			(camera.kinectMarkers[i]).DrawCoordinateSys();
		}
	}

	// draw image
	while(!sns_cx.shutdown)
	{
		// grab a new frame
		if(mode != ACH)
		{
			if( !camera.capture.grab() )
			{
				cout << "Can not grab images." << endl;
				return -1;
			}
		}

		// process video image and draw scene
		camera.UpdateScene(false);

		// detect the marker in the scene
		camera.DetectMarkers(true);


		// do kinect processing
		if(mode == KINECT)
		{
			camera.CreatePointCloud();

		}
		// send the data
		camera.SendMsg(32);

		// fire off the cull and draw traversals of the scene.
		if(useGraphics)
			viewer->frame();

		// process keyboard input
		if( waitKey( 30 ) >= 0 )
			exit(0);

		// release memory
		aa_mem_region_local_release();
	}

	return 0;
}
