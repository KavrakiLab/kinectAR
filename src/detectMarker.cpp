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

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

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



int main(int argc, char **argv)
{
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
	osg::ShapeDrawable* shape  = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),70.0f,40.0f,0.05f));
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
	}
	
	// channel name
	kinect.OpenChannel(argv[1]);
			
	// draw image
	while(true)
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
