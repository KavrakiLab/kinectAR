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

#include <AR/gsub.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/video.h>

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

using namespace cv;
using namespace std;


KinectAR kinect;


const char *opt_channel = "marker";


int main(int argc, char **argv)
{
	for( int c; -1 != (c = getopt(argc, argv, "c:?" SNS_OPTSTRING)); ) {
		switch(c) {
			SNS_OPTCASES
		case 'c':
			opt_channel = optarg;
			break;
		case '?':   /* help     */
		default:
			puts( "Usage: detect_marker -c channel\n"
			      "Detect markers with kinect\n"
			      "\n"
			      "Options:\n"
			      "  -c CANNEL,                   Set output Ach channel\n"
			      "  -?,                          Give program help list\n"
			      "\n"
			      "Report bugs to <hbenamor@cc.gatech.edu>" );
		}
	}



	// use an ArgumentParser object to manage the program arguments.
	int numArgs = 6;
	char* args[6];
	args[0]= "program";
	args[1]= "--window";
	args[2]= "100";
	args[3]= "100";
	args[4]= "400";
	args[5]= "400";
	osg::ArgumentParser arguments(&numArgs, args);
	osgViewer::Viewer viewer(arguments);

	// create the root node of the scenegraph
	osg::Group *root = new osg::Group;

	// add the table
	osg::Geode* table 	 = new osg::Geode;
	osg::ShapeDrawable* shape  = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),80.0f,40.0f,0.05f));
	osg::PositionAttitudeTransform* transf = new osg::PositionAttitudeTransform;

	transf->setPosition(osg::Vec3(0,0,120));
	root->addChild(transf);
	transf->addChild(table);
	table->addDrawable(shape);

	// add a viewport to the viewer and attach the scene graph.
	viewer.setSceneData(root);

	// set the trackball manipulator
	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	viewer.realize();

	double alpha = 0.0;

	// add to the sceneGraph
	for(int i = 0; i < 32; i++)
	{
		(kinect.kinectMarkers[i]).AddToSceneGraph(root);
		(kinect.kinectMarkers[i]).DrawCoordinateSys();
	}

	// channel name
	kinect.OpenChannel(opt_channel);

	// draw image
	while(!sns_cx.shutdown)
	{
		// autoposition wird inkrementiert
		alpha+=0.01;

		// fire off the cull and draw traversals of the scene.
		viewer.frame();

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
			kinect.SendMsg(32);
		}

		if( waitKey( 30 ) >= 0 )
			kinect.Keyboard('n', 1, 1);
		//break;
	}

	return 0;
}
