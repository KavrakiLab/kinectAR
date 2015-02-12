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

using namespace std;
using namespace cv;

class Settings {
/**
 * Container class for various settings used during camera calibration.
 * Initialized with default values but they can be changed.
 */
public:
	int boardWidth;		// Dimensions of chessboard
	int boardLength;
	float squareSize;	// Square size in centimeters
	string outputFileName;	// Output XML file to save calibration to
	int flag;		// Flags for calibration

	Settings() {
		// Set default values
		boardWidth = 8;
		boardLength = 6;

		squareSize = 10.0;

		outputFileName = "calibration.xml";

		flag = 0;
		flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
		flag |= CV_CALIB_ZERO_TANGENT_DIST;
		flag |= CV_CALIB_FIX_ASPECT_RATIO;
	}


	/**
	 * Returns Size representing chessboard dimensions
	 */
	Size getBoardSize(void) {
		return Size(boardWidth, boardLength);
	}
};


class IMsg {
/**
 * Communication class to send integer values over an Ach channel.
 */
public:
	ach_channel_t chan;	// Ach communication channel
	int message;		// Container for the message


	IMsg(const char *channelName) {
		sns_chan_open(&chan, channelName, NULL);
	}


	/**
	 * Send an integer message 
	 */
	void sendMessage(int msg) {
		SNS_LOG(
			LOG_DEBUG,
			"Sending message %d!",
			msg 
		);

		ach_status_t r = ach_put(&chan, &msg, sizeof(int));

		SNS_REQUIRE(
			ACH_OK == r,
			"Could not send message! %s\n",
			ach_result_to_string(r) 
		);
	}


	/**
	 * Receive the latest message. Does not wait or block.
	 */
	int recvMessage(void) {
		size_t messageSize;

		int *messagePtr;

		ach_status_t r = sns_msg_local_get(
			&chan,
			(void **) &messagePtr,
			&messageSize,
			NULL,
			(ach_get_opts_t) (ACH_O_LAST | ACH_O_WAIT)
		);

		switch (r) {
		case ACH_CANCELED: {
			return 0;
		}
		case ACH_OK:
		case ACH_MISSED_FRAME:
		case ACH_STALE_FRAMES: {
			break;
		}
		default: {
			SNS_DIE(
				"Could not receive signal: %s\n",
				ach_result_to_string(r)
			);
		}
		}

		message = *messagePtr;

		return message;
	}
};