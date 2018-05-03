#ifndef __GPSDATASTRUCT_H__
#define __GPSDATASTRUCT_H__

#include <iostream>


class GPSDataStruct
{
public:
	GPSDataStruct();
	virtual ~GPSDataStruct();

	double latitude;
	double longitude;
	double altitude;

	friend std::ostream& operator<< (std::ostream &os, const GPSDataStruct &gps)
	{
		os << gps.longitude << " " << gps.latitude << " " << gps.altitude;
		return os;
	}
};

#endif // __GPSDATASTRUCT_H__