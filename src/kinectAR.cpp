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

using namespace cv;
using namespace std;

static inline size_t msg_size( size_t n ) {
	return sizeof(struct sendMarker) - sizeof(struct tf_qv) + (n) * sizeof(struct tf_qv);
}

uint64_t  mask_set_i(uint64_t mask, uint8_t i, int is_visible)
{
	return is_visible ? mask | (1<<i) : mask;

}

#define ALLOCA_MSG(n) ( (struct sendMarker*)alloca( msg_size(n) ) )

KinectAR::KinectAR(const char* calibFileName, CParams p, const char *chan_name_cam, const char *chan_name_tf)
{
	int imageMode;
	camMode = ACH;
	params  = p;

	image = cvCreateImage(cvSize(params.getResX(), params.getResY()), IPL_DEPTH_8U, 3);

	rec.init(chan_name_cam, params.getResX(), params.getResY());

	sns_chan_open( &channel_tf, chan_name_tf, NULL );

	bool modeRes=false;


	capture.set(CV_CAP_PROP_FRAME_WIDTH, params.getResX());
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, params.getResY());

	// create the markers to be tracked
	for(int i = 0; i < 32; i++)
	{
		// create a bunch of markers
		kinectMarkers.push_back(KinectMarker());
	}

	// finished initialization
	init = true;
	calib = calibFileName;

	// set marker size
	marker_detector.SetMarkerSize(params.getMarkerSize(), 5, 2);

	// set the size of individual, aka bigger, markers
	std::map<int, double> isizes = params.getIndivMarkerSize();

	for(std::map<int, double>::iterator iter = isizes.begin(); iter != isizes.end(); iter++)
	{
		// for marker ids larger than 255, set the content resolution accordingly
		marker_detector.SetMarkerSizeForId(iter->first, iter->second);
	}
}

// KinectAR::KinectAR(CamMode mode, char* calibFileName, CParams p)
// {
//	int imageMode;
//	camMode = mode;
//	params  = p;

//	// using the kinect
//	if(camMode == KINECT)
//	{
//		image = cvCreateImage(cvSize(1280,1024), IPL_DEPTH_8U, 3);
//		std::cout << "Kinect Device opening ..." << std::endl;
//		capture.open( CV_CAP_OPENNI );
//		std::cout << "done." << std::endl;
//	}
//	else
//	{
//		image = cvCreateImage(cvSize(params.getResX(), params.getResY()), IPL_DEPTH_8U, 3);

//		// using the web cam
//		if (camMode == NORMAL)
//		{
//			capture.open( 0 );

//			// did not find any camera
//			if( !capture.isOpened() )
//			{
//				std::cout << "Can not open a capture object." << std::endl;
//				exit(0);
//			}
//		}
//		// using ach
//		else if (camMode == ACH)
//		{
//			const char* tmpChannel = params.getAchInputChannel().c_str();
//			rec.init(tmpChannel, params.getResX(), params.getResY());
//		}
//	}

//	bool modeRes=false;

//	if(camMode == KINECT)
//		modeRes = capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_SXGA_15HZ );

//	else
//	{
//		capture.set(CV_CAP_PROP_FRAME_WIDTH, params.getResX());
//		capture.set(CV_CAP_PROP_FRAME_HEIGHT, params.getResY());
//	}

//	// create the markers to be tracked
//	for(int i = 0; i < 32; i++)
//	{
//		// create a bunch of markers
//		kinectMarkers.push_back(KinectMarker());
//	}

//	// finished initialization
//	init = true;
//	calib = calibFileName;

//	// set marker size
//	marker_detector.SetMarkerSize(params.getMarkerSize(), 5, 2);

//	// set the size of individual, aka bigger, markers
//	std::map<int, double> isizes = params.getIndivMarkerSize();

//	for(std::map<int, double>::iterator iter = isizes.begin(); iter != isizes.end(); iter++)
//	{
//		// for marker ids larger than 255, set the content resolution accordingly
//		marker_detector.SetMarkerSizeForId(iter->first, iter->second);
//	}
// }

void KinectAR::DetectMarkers(bool print)
{
	// get the current video image
	image->imageData = (char *) bgrImage.data;

	if (init)
	{
		init = false;
		if(camMode == KINECT)
		{
			// no calibration needed in this case
			cam.SetRes(image->width, image->height);
		}
		else
		{
			// load calibration data
			cam.SetCalib(calib, image->width, image->height);
			std::cout<<" [Loading Camera Calibration Successful!]"<< std::endl;
		}
	}

	// for marker ids larger than 255, set the content resolution accordingly
	marker_detector.Detect(image, &cam, false, false);

	//std::cout << image->imageData.size << std::endl;
	for (size_t i=0; i<marker_detector.markers->size(); i++)
	{
		kinectMarkers[i].Update(&(*(marker_detector.markers))[i]);

		int id = (*(marker_detector.markers))[i].GetId();

		//SNS_LOG( LOG_ERR, "Detected Num: %d\n", id );

		// print out the current pose
		if(print) kinectMarkers[i].PrintAlvarPose();
	}
}

void KinectAR::UpdateScene(bool draw)
{
	// get image from camera
	if(camMode == KINECT)
	{
		capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE );
		capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
	}
	else if (camMode == ACH)
	{
		// streaming of pictures
		bgrImage = rec.receiveImage();
	}
	else
	{
		capture >> bgrImage;
	}

	// only draw if corresponding flag is set
	if(draw)
		imshow( "rgb image", bgrImage );
}

void KinectAR::OpenChannel(const char* chan_name_cam, const char *chan_name_tf)
{
	if( chan_name_tf ) sns_chan_open( &channel_tf, chan_name_tf, NULL );
}

void KinectAR::SendMsg(size_t n)
{
	// set the number of objects
	struct timespec now = sns_now();

	sns_msg_wt_tf *msg;
	//if( KINECT == camMode ) {
	//	msg = sns_msg_wt_tf_local_alloc(n*2);
	//} else {
	msg = sns_msg_wt_tf_local_alloc(n);
	//}
	sns_msg_set_time( &msg->header, &now, 0 );

	// loop over all visibile markers
	alvar::Pose p;
	double tmp[4];
	CvMat mat = cvMat(4, 1, CV_64F, tmp);
	for (size_t i=0; i<marker_detector.markers->size(); i++)
	{
		int id = (*(marker_detector.markers))[i].GetId();
		if( id >= n ) {
			SNS_LOG( LOG_ERR, "Invalid id: %d\n", id );
			continue;
		}

		sns_wt_tf *wt_tfA = &(msg->wt_tf[id]);

		p = (*(marker_detector.markers))[i].pose;

		// get the quaternion orientation
		p.GetQuaternion(&mat);
		double* alvar_quat = (double*)mat.data.ptr;

		// set visibility
		wt_tfA->weight = 1.0 - (*(marker_detector.markers))[i].GetError();

		// 1.) set data of ALVAR
		// set the positions
		for(int j = 0; j < 3; j++)
			wt_tfA->tf.v.data[j] = p.translation[j] * 1e-2;

		// set the orientation
		wt_tfA->tf.r.w = alvar_quat[0];
		wt_tfA->tf.r.x = alvar_quat[1];
		wt_tfA->tf.r.y = alvar_quat[2];
		wt_tfA->tf.r.z = alvar_quat[3];

		// 2.) set the data of Kinect
		// set the positions
		/*if(camMode == KINECT)
		{
			assert( n*2 == msg->header.n );
			sns_wt_tf *wt_tfK = &(msg->wt_tf[id+n]);
			Eigen::Vector3f kpos  = kinectMarkers[i].GetKinectPos();
			double* kquat = kinectMarkers[i].GetKinectQuat();

			for(int j = 0; j < 3; j++)
				wt_tfK->tf.v.data[j] = kpos[j] * 1e-2;

			// set the orientation
			wt_tfK->tf.r.w = kquat[3];
			wt_tfK->tf.r.x = kquat[0];
			wt_tfK->tf.r.y = kquat[1];
			wt_tfK->tf.r.z = kquat[2];

			// set weight of marker
			wt_tfK->weight = 1.0 - (*(marker_detector.markers))[i].GetError();
		} else {*/
		assert( n == msg->header.n );
		//}
	}

	// send out the message via ACH
	enum ach_status r = sns_msg_wt_tf_put( &channel_tf, msg );
	if( ACH_OK != r )
	{
		syslog( LOG_ERR, "Could not put data: %s\n", ach_result_to_string(r) );
	}

	// some debug messages
	if(marker_detector.markers->size() > 0 &&
		 sns_cx.verbosity )
	{
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

void KinectAR::CreatePointCloud()
{
	// LEAVE THIS! Prints out the distance matrix
	/*for(int i = 0; i < show.rows; i++)
	{
		for(int j = 0; j < show.cols; j++)
		{

			int d = (int) show.at<uchar>(i,j);
			file << d << " "; // << std::endl;
		}
		file << std::endl;
	}
	file.close();*/

	//markerPoints2D.clear();
	capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
	for(int k=0; k < (*(marker_detector.markers)).size(); k++)
	{
		kinectMarkers[k].CalculatePointCloud(depthMap);
		kinectMarkers[k].CalculateCorner3D(depthMap);

		kinectMarkers[k].GetNormalVector();
	}
}


void KinectAR::Keyboard(int key, int x, int y)
{
	switch(key)
	{
		case 'n':
		CreatePointCloud();
		break;
	}
	std::cout << "Key pressed!" << std::endl;
}
