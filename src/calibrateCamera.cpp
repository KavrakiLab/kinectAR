/**
 * Adapted from http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
 */

/* -*- mode: C++; c-basic-offset: 8; indent-tabs-mode: t;  -*- */
#include <amino.h>
#include <ach.h>
#include <sns.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "imageReceiver.h"
#include "calibrateCamera.h"

using namespace cv;
using namespace std;

bool findBoard(Mat *image, Settings &settings, vector<Point2f> &pointBuf) {
	Size imageSize = image->size();

	bool found = findChessboardCorners(
			*image,
			settings.getBoardSize(),
			pointBuf,
			CV_CALIB_CB_ADAPTIVE_THRESH
			| CV_CALIB_CB_FAST_CHECK
			| CV_CALIB_CB_NORMALIZE_IMAGE
		);

	if (found) {
		// Improve estimate
		Mat viewGray;

		cvtColor(*image, viewGray, COLOR_BGR2GRAY);
		cornerSubPix(
			viewGray,
			pointBuf,
			Size(11, 11),
			Size(-1,-1),
			TermCriteria(
				CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
				30,
				0.1
			)
		);
	}

	return found;
}

void calcBoardCornerPositions(Settings &settings, vector<Point3f> &corners) {
	corners.clear();

	Size boardSize = settings.getBoardSize();
	float squareSize = settings.squareSize;

	int i;
	for(i = 0; i < boardSize.height; ++i) {
		int j;
		for(j = 0; j < boardSize.width; ++j) {
			corners.push_back(
				Point3f(
					float(j * squareSize),
					float(i * squareSize),
					0
				)
			);
		}
	}
}

void saveCalibration(Settings &settings, Size &imageSize, Mat &cameraMatrix,
 Mat &distCoeffs) {
	FileStorage fs(settings.outputFileName, FileStorage::WRITE);

	fs << "width" << imageSize.width;
	fs << "height" << imageSize.height;

	fs << "intrinsic_matrix" << cameraMatrix;
	fs << "distortion" << distCoeffs;
}

bool generateCalibration(Settings &settings, Size imageSize,
 vector<vector<Point2f>> imagePoints) {
	Mat cameraMatrix, distCoeffs;

	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;

	cameraMatrix = Mat::eye(3, 3, CV_64F);

	if(settings.flag & CV_CALIB_FIX_ASPECT_RATIO) {
		cameraMatrix.at<double>(0, 0) = 1.0;
	}

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f>> objectPoints(1);

	calcBoardCornerPositions(settings, objectPoints[0]);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(
		objectPoints,
		imagePoints,
		imageSize,
		cameraMatrix,
		distCoeffs,
		rvecs,
		tvecs,
		settings.flag | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5
	);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	if (ok) {
		saveCalibration(
			settings,
			imageSize,
			cameraMatrix,
			distCoeffs
		);
	}

	return ok;
}


void calibrateLoop(Settings &settings, IMsg &msgComm, ImageReceiver &camera) {
	vector<vector<Point2f>> imagePoints;
	Size imageSize;

	while (!sns_cx.shutdown) {
		int msg = msgComm.recvMessage();

		// We only care about messages less than zero
		if (msg >= 0) {
			continue;
		}

		switch (msg) {
		case (-1): {
			cout << "Trying to find calibration board..." << endl;
			// Capture image and use for calibration
			Mat *image = camera.receiveImage2(NULL);
			imageSize = image->size();

			vector<Point2f> pointBuf;

			bool found = findBoard(image, settings, pointBuf);

			if (found) {
				imagePoints.push_back(pointBuf);
				cout << "Board found! Image saved." << endl;

			} else {
				cout << "Board not found." << endl;
			}

			break;
		}
		case (-2): {
			if (imagePoints.size() == 0) {
				cout << "No images to use!" << endl;
				break;
			}

			bool ok = generateCalibration(
				settings,
				imageSize,
				imagePoints
			);

			if (ok) {
				cout << "Calibration file generated!" << endl;

			} else {
				cout << "Uh-oh! No file made." << endl;
			}

			break;
		}
		}
		msgComm.sendMessage(imagePoints.size());
	}
}


int main(int argc, char *argv[]) {
	sns_init();

	Settings settings;

	char *communcationChannelName = NULL;
	char *cameraChannelName = NULL;

	int c;
	opterr = 0;
	while ((c = getopt(argc, argv, "w:l:s:o:c:m:?")) != -1) {
		switch (c) {
		case ('w'): {
			settings.boardWidth = atoi(optarg);
			break;
		}
		case ('l'): {
			settings.boardLength = atoi(optarg);
			break;
		}
		case ('s'): {
			settings.squareSize = atof(optarg);
			break;
		}
		case ('o'): {
			settings.outputFileName = optarg;
			break;
		}
		case ('c'): {
			cameraChannelName = optarg;
			break;
		}
		case ('m'): {
			communcationChannelName = optarg;
			break;
		}
		case ('?'):
		default: {
			puts(
				"Camera Calibration for Ach streaming camera\n"
				"Required:\n"
				"-c Camera Ach channel\n"
				"-m Message Ach channel\n"
				"Options:\n"
				"-w Calibration chessboard width\n"
				"-l Calibration chessboard length\n"
				"-s Calibration chessboard square size in cm\n"
				"-o Calibration output file name\n"
			);
			return -1;
		}
		}
	}

	if (!cameraChannelName) {
		puts("Bad camera channel name! Check help with -?\n");
		return -1;
	}

	if (!communcationChannelName) {
		puts("Bad communication channel name! Check help with -?\n");
		return -1;
	}

	ImageReceiver camera(cameraChannelName);	// Camera stream
	IMsg msgComm(communcationChannelName);		// Message channel

	{
		ach_channel_t *chans[] = { &camera.chan, &msgComm.chan, NULL };
		sns_sigcancel(chans, sns_sig_term_default);
	}

	sns_start();

	calibrateLoop(settings, msgComm, camera);

	return 0;
}