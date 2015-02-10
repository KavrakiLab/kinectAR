
// in - ach buffer to read for images, ach buffer to read for signals
// out - xml file to write parameters into

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

using namespace cv;
using namespace std;

class Settings
{
public:
	Settings() : goodInput(false) {

	}

	enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
	enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

	void write(FileStorage& fs) const {
		fs << "{" << "BoardSize_Width"  << boardSize.width
				  << "BoardSize_Height" << boardSize.height
				  << "Square_Size"         << squareSize
				  << "Calibrate_Pattern" << patternToUse
				  << "Calibrate_NrOfFrameToUse" << nrFrames
				  << "Calibrate_FixAspectRatio" << aspectRatio
				  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
				  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

				  << "Write_DetectedFeaturePoints" << bwritePoints
				  << "Write_extrinsicParameters"   << bwriteExtrinsics
				  << "Write_outputFileName"  << outputFileName

				  << "Show_UndistortedImage" << showUndistorsed

				  << "Input_FlipAroundHorizontalAxis" << flipVertical
		   << "}";
	}

	void read(const FileNode& node) {
		node["BoardSize_Width" ] >> boardSize.width;
		node["BoardSize_Height"] >> boardSize.height;
		node["Calibrate_Pattern"] >> patternToUse;
		node["Square_Size"]  >> squareSize;
		node["Calibrate_NrOfFrameToUse"] >> nrFrames;
		node["Calibrate_FixAspectRatio"] >> aspectRatio;
		node["Write_DetectedFeaturePoints"] >> bwritePoints;
		node["Write_extrinsicParameters"] >> bwriteExtrinsics;
		node["Write_outputFileName"] >> outputFileName;
		node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
		node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
		node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
		node["Show_UndistortedImage"] >> showUndistorsed;
		interpret();
	}

	void interpret() {
		goodInput = true;

		if (boardSize.width <= 0 || boardSize.height <= 0) {
			cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
			goodInput = false;
		}

		if (squareSize <= 10e-6) {
			cerr << "Invalid square size " << squareSize << endl;
			goodInput = false;
		}

		if (nrFrames <= 0) {
			cerr << "Invalid number of frames " << nrFrames << endl;
			goodInput = false;
		}

		flag = 0;
		if(calibFixPrincipalPoint)
			flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
		if(calibZeroTangentDist)
			flag |= CV_CALIB_ZERO_TANGENT_DIST;
		if(aspectRatio)
			flag |= CV_CALIB_FIX_ASPECT_RATIO;


		calibrationPattern = NOT_EXISTING;

		if (!patternToUse.compare("CHESSBOARD"))
			calibrationPattern = CHESSBOARD;

		if (!patternToUse.compare("CIRCLES_GRID"))
			calibrationPattern = CIRCLES_GRID;

		if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID"))
			calibrationPattern = ASYMMETRIC_CIRCLES_GRID;

		if (calibrationPattern == NOT_EXISTING) {
			cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
			goodInput = false;
		}
	}

	static bool readStringList(const string& filename, vector<string>& l) {
		l.clear();
		FileStorage fs(filename, FileStorage::READ);
		if( !fs.isOpened() )
			return false;
		FileNode n = fs.getFirstTopLevelNode();
		if( n.type() != FileNode::SEQ )
			return false;
		FileNodeIterator it = n.begin(), it_end = n.end();
		for( ; it != it_end; ++it )
			l.push_back((string)*it);
		return true;
	}

public:
	Size boardSize;					// The size of the board -> Number of items by width and height
	Pattern calibrationPattern;		// One of the Chessboard, circles, or asymmetric circle pattern
	float squareSize;				// The size of a square in your defined unit (point, millimeter,etc).
	int nrFrames;					// The number of frames to use from the input for calibration
	float aspectRatio;				// The aspect ratio
	bool bwritePoints;				//  Write detected feature points
	bool bwriteExtrinsics;			// Write extrinsic parameters
	bool calibZeroTangentDist;		// Assume zero tangential distortion
	bool calibFixPrincipalPoint;	// Fix the principal point at the center
	bool flipVertical;				// Flip the captured images around the horizontal axis
	string outputFileName;			// The name of the file where to write
	bool showUndistorsed;			// Show undistorted images after calibration

	bool goodInput;
	int flag;

private:
	string patternToUse;
};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
	if(node.empty())
		x = default_value;
	else
		x.read(node);
}

class Signal
{
public:	
	ach_channel_t chan;

	Signal(const char *channelName) {
		sns_chan_open(&chan, channelName, NULL);
	}

	void sendSignal(int sig) {
		SNS_LOG( LOG_DEBUG, "Sending Signal %d", sig );

		ach_status_t r = ach_put(&chan, &sig, sizeof(int));

		SNS_REQUIRE( ACH_OK == r, "Could not signal! %s\n", ach_result_to_string(r) );
	}

	int recvSignal(void) {
		size_t signalSize;
		int *sig;
		ach_status_t r = sns_msg_local_get( &chan, (void**) &sig, &signalSize,
							NULL, (ach_get_opts_t) (ACH_O_LAST) );
		switch( r ) {
		case ACH_CANCELED:
			return 0;
		case ACH_OK:
		case ACH_MISSED_FRAME:
		case ACH_STALE_FRAMES:
			break;
		default:
			SNS_DIE( "Could not receive signal: %s\n",
				 ach_result_to_string(r) );
		}

		// cout << *sig << endl;
		return *sig;
	}
};

int calibrate(Mat *m, Settings *s, vector<Point2f> *v) {
	Size imageSize = m->size();

	if (s->flipVertical) {
		flip(*m, *m, 0);
	}

	bool found;
	switch( s->calibrationPattern ) // Find feature points on the input format
	{
	case Settings::CHESSBOARD:
		found = findChessboardCorners( *m, s->boardSize, *v,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		break;
	case Settings::CIRCLES_GRID:
		found = findCirclesGrid( *m, s->boardSize, *v );
		break;
	case Settings::ASYMMETRIC_CIRCLES_GRID:
		found = findCirclesGrid( *m, s->boardSize, *v, CALIB_CB_ASYMMETRIC_GRID );
		break;
	default:
		found = false;
		break;
	}

	if (found) {
		// improve the found corners' coordinate accuracy for chessboard
		if(s->calibrationPattern == Settings::CHESSBOARD) {
			Mat viewGray;
			cvtColor(*m, viewGray, COLOR_BGR2GRAY);
			cornerSubPix( viewGray, *v, Size(11,11),
				Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
		}

		// Draw the corners.
		drawChessboardCorners( *m, s->boardSize, Mat(*v), found );
		return 1;
	} else {
		return 0;
	}

}

int main( int argc, char* argv[] )
{
	ach_channel_t signalChan;

	sns_init();
	if(argc < 3) {
		std::cout << "Usage: calibrateCamera camera_channel signal_channel calibration_file" << std::endl;
		return 0;
	}

	ImageReceiver rec(argv[1]);

	Signal sig(argv[2]);

	Settings s;
	const string inputSettingsFile = argv[3];
	FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings

	if (!fs.isOpened()) {
		cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
		return -1;
	}
	fs["Settings"] >> s;
	fs.release();

	{
		ach_channel_t *chans[] = {&rec.chan, &signalChan, NULL};
		sns_sigcancel( chans, sns_sig_term_default );
	}

	sns_start();

	bool save = false;
	Mat *ms = NULL;

	vector<vector<Point2f> > imagePoints;

	while(!sns_cx.shutdown) {
		Mat *m = rec.receiveImage2(NULL);
		bool blinkOutput = false;

		int sg = sig.recvSignal();
		switch (sg) {
		case (0): {
			break;
		}
		case (1): {
			save = true;
			ms = m;
			sig.sendSignal(0);
			break;
		}
		case (2): {
			if (!save) {
				cout << "No image saved!" << endl;
				break;
			}
			vector<Point2f> pointBuf;
			int r = calibrate(m, &s, &pointBuf);

			if (r) {
				imagePoints.push_back(pointBuf);
				blinkOutput = true;
				sig.sendSignal(3);
			} else {
				cout << "Cannot find calibration board!" << endl;
			}
			break;
		}
		case (3): {
			save = false;
			if (ms) {
				delete ms;
				ms = NULL;
			}
			sig.sendSignal(0);
			break;
		}
		}

		Mat *im = NULL;
		if (save) {
			im = ms;
		} else {
			im = m;
		}

		if (im) {

			if(blinkOutput) {
				bitwise_not(*im, *im);
			}

			const Scalar RED(0, 0, 255);
			string msg = format("%d Images Captured", (int) imagePoints.size());


			int baseline = 0;
			Size textSize = getTextSize(msg, 1, 1, 1, &baseline);
			Point textOrigin(im->cols - 2 * textSize.width - 10, im->rows - 2 * baseline - 10);
			putText( *im, msg, textOrigin, 1, 1, RED);

			imshow( "Calibration", *im );
		}

		if (!save) {
			if (m) {
				delete m;
			}
		}

		char key = waitKey( 30 );
		if (key != -1) {
			cout << key << endl;

			switch (key) {
			case ('s'): {
				sig.sendSignal(1);
				break;
			}
			case ('c'): {
				sig.sendSignal(2);
				break;
			}
			case ('d'): {
				sig.sendSignal(3);
				break;
			}
			}
		}

		aa_mem_region_local_release();
	}
	return 0;
}
