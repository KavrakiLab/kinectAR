
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
	Size boardSize;				// The size of the board -> Number of items by width and height
	Pattern calibrationPattern;		// One of the Chessboard, circles, or asymmetric circle pattern
	float squareSize;			// The size of a square in your defined unit (point, millimeter,etc).
	int nrFrames;				// The number of frames to use from the input for calibration
	float aspectRatio;			// The aspect ratio
	bool bwritePoints;			//  Write detected feature points
	bool bwriteExtrinsics;			// Write extrinsic parameters
	bool calibZeroTangentDist;		// Assume zero tangential distortion
	bool calibFixPrincipalPoint;		// Fix the principal point at the center
	bool flipVertical;			// Flip the captured images around the horizontal axis
	string outputFileName;			// The name of the file where to write
	bool showUndistorsed;			// Show undistorted images after calibration

	bool goodInput;
	int flag;

private:
	string patternToUse;
};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings()) {
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
		ach_status_t r = sns_msg_local_get(
			&chan,
			(void**) &sig,
			&signalSize,
			NULL,
			(ach_get_opts_t) (ACH_O_LAST)
		);

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

int findRegion(Mat *m, Settings *s, vector<Point2f> *v) {
	Size imageSize = m->size();

	if (s->flipVertical) {
		flip(*m, *m, 0);
	}

	bool found;
	switch( s->calibrationPattern )
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

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
	vector<vector<Point2f> > imagePoints );

static bool calibrate(Settings& s, Size imageSize, vector<vector<Point2f> > imagePoints ) {
	Mat cameraMatrix, distCoeffs;

	return runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
}



int main(int argc, char* argv[]) {
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
		cout << "Could not open configuration file: \"" << inputSettingsFile << "\"" << endl;
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
	bool found = false;

	Mat *m = NULL;
	Size imageSize;

	const Scalar RED(0, 0, 255);
	const Scalar GREEN(0, 255, 0);

	string msg = "";

	vector<Point2f> pointBuf;
	vector<vector<Point2f> > imagePoints;

	while(!sns_cx.shutdown) {
		bool blinkOutput = false;

		int sg = sig.recvSignal();
		switch (sg) {
		case (0): {
			break;
		}
		case (1): {
			save = true;
			sig.sendSignal(0);
			break;
		}
		case (2): {
			if (!save) {
				msg = "No image saved!";
				break;
			}
			if (!found) {
				msg = "No board found!";
				break;
			}

			// Save this one
			imagePoints.push_back(pointBuf);
			blinkOutput = true;

			break;
		}
		case (3): {
			save = false;
			found = false;
			sig.sendSignal(0);
			msg = "Frame destroyed.";
			break;
		}
		case (4): {
			if (imagePoints.size() > 0) {
				if (calibrate(s, imageSize, imagePoints)) {
					msg = "Generation Success!";
				} else {
					msg = "Generation Failure!";
				}
			} else {
				msg = "No images saved!";
			}
			break;
		}
		}

		if (!save) {
			if (m) {
				delete m;
			}
			m = rec.receiveImage2(NULL);
			imageSize = m->size();
		} else {
			if (!found) {
				msg = "Attempting to find board...";
				found = findRegion(m, &s, &pointBuf);

				if (found) {
					msg = "Found board!";
				}
			}
		}

		if (m) {
			Mat copyM;
			m->copyTo(copyM);

			if(blinkOutput) {
				bitwise_not(copyM, copyM);
			}

			int baseline = 0;
			string imageCountMsg = format("%d Images Captured", (int) imagePoints.size());

			Size textSize = getTextSize(imageCountMsg, 1, 1, 1, &baseline);
			Point bottomLeft(10, copyM.rows - 2 * baseline - 10);
			Point bottomRight(copyM.cols - 2 * textSize.width - 10, copyM.rows - 2 * baseline - 10);

			Scalar textColor = (found) ? GREEN : RED;
			putText(copyM, imageCountMsg, bottomRight, 1, 1, textColor);
			putText(copyM, msg, bottomLeft, 1, 1, textColor);

			putText(copyM, "Controls -", Point(10, 15), 1, 1, textColor);
			putText(copyM, "s - Saves current frame for processing", Point(10, 30), 1, 1, textColor);
			putText(copyM, "d - Deletes saved frame", Point(10, 45), 1, 1, textColor);
			putText(copyM, "u - Uses current saved frame for calibration", Point(10, 60), 1, 1, textColor);
			putText(copyM, "g - Generates camera calibration file", Point(10, 75), 1, 1, textColor);
			putText(copyM, "Text turns green if valid frame found.", Point(10, 90), 1, 1, textColor);

			imshow( "Calibration", copyM );
		}
		char key = waitKey(30);

		if (key != -1) {
			switch (key) {
			case ('s'): {
				sig.sendSignal(1);
				break;
			}
			case ('u'): {
				sig.sendSignal(2);
				break;
			}
			case ('d'): {
				sig.sendSignal(3);
				break;
			}
			case ('g'): {
				sig.sendSignal(4);
				break;
			}
			case (27): {
				aa_mem_region_local_release();
				exit(0);
			}
			}
		}

		aa_mem_region_local_release();
	}
	return 0;
}


static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
        break;
    default:
        break;
    }
}

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    // cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;

    if( s.flag )
    {
        sprintf( buf, "flags: %s%s%s%s",
            s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    // cout << (ok ? "Calibration succeeded" : "Calibration failed")
    //     << ". avg re projection error = "  << totalAvgErr ;

    if( ok )
        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                            imagePoints, totalAvgErr);
    return ok;
}
