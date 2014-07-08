/* -*- mode: C++; c-basic-offset: 8; indent-tabs-mode: t;  -*- */
#ifndef CPARAMS_H
#define CPARAMS_H

#include <iostream>
#include <fstream>

class CParams
{
public:
	// loads all params needed for tracking
	CParams() :
		resX(1920),
		resY(1080),
		markerSize(2.8),
		achOutputChannel("markers"),
		achInputChannel("video1")
	{
		// set default params
	}

	// load params from file
	CParams(const char* filename)
	{
		loadParams(filename);
	}

	// set resolution
	void setResolution(int x, int y)
	{
		resX = x;
		resY = y;
	}

	// set the ach channel for publishing marker positions
	void setAchOutputChannel(std::string channel)
	{
		achOutputChannel = channel;
	}

	// set the ach channel for reading camera images
	void setAchInputChannel(std::string channel)
	{
		achInputChannel = channel;
	}

	// get the ach channel for publishing marker positions
	std::string getAchOutputChannel()
	{
		return achOutputChannel;
	}

	// get the ach channel for publishing marker positions
	std::string getAchInputChannel()
	{
		return achInputChannel;
	}

	// return the width of the image
	int getResX()
	{
		return resX;
	}

	// return the height of the image
	int getResY()
	{
		return resY;
	}

	// set the size of a marker
	void setMarkerSize(double s)
	{
		markerSize = s;
	}

	// get the size of a marker
	double getMarkerSize()
	{
		return markerSize;
	}

	// get the marker size of individual markers
	// that are bigger than the rest
	std::map<int, double> getIndivMarkerSize()
	{
		return  indivMarkerSizes;
	}

	// load parameters
	void loadParams(const char* file)
	{
		std::ifstream input(file);
		std::cout << "->Loading Parameters!" << std::endl;
		if(input.is_open())
		{
			// resolution
			input >> resX;
			input >> resY;

			// input/output channel
			input >> achInputChannel;
			input >> achOutputChannel;

			// overall marker size
			input >> markerSize;

			int id;
			double size;
			// individual marker sizes
			while(!input.eof())
			{
				// get id of marker
				input >> id;
				input >> size;

				indivMarkerSizes[id] = size;
			}

			input.close();
		}
	}

protected:
	int resX, resY;
	double markerSize;
	std::map<int, double> indivMarkerSizes;
	std::string achOutputChannel;
	std::string achInputChannel;
};

#endif
