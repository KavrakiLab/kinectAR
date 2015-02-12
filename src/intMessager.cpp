#include <amino.h>
#include <ach.h>
#include <sns.h>

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <unistd.h>

#include "calibrateCamera.h"

int main(int argc, char *argv[]) {
	sns_init();

	char *communcationChannelName = NULL;

	bool toSend = false;
	int message = 0;

	int c;
	opterr = 0;
	while ((c = getopt(argc, argv, "m:i:?")) != -1) {
		switch (c) {
		case ('m'): {
			communcationChannelName = optarg;
			break;
		}
		case ('i'): {
			message = atoi(optarg);
			toSend = true;
			break;
		}
		case ('?'):
		default: {
			puts(
				"Integer Messaging for Ach channel\n"
				"If not provided an integer to send, will wait"
				" for a message to appear and return the value"
				" of the message as exit code.\n"
				"Required:\n"
				"-m Message Ach channel\n"
				"Optional:\n"
				"-i Integer to send\n"
			);
			return -1;
		}
		}
	}

	if (!communcationChannelName) {
		puts("Bad communication channel name! Check help with -?\n");
		return -1;
	}

	IMsg msgComm(communcationChannelName);	// Message channel

	{
		ach_channel_t *chans[] = { &msgComm.chan, NULL };
		sns_sigcancel(chans, sns_sig_term_default);
	}

	sns_start();

	if (toSend) {
		msgComm.sendMessage(message);
		return 0;

	} else {
		return msgComm.recvMessage();
	}
}