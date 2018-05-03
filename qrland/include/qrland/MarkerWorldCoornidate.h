#ifndef __MarkerWorldCoornidate_h__
#define __MarkerWorldCoornidate_h__

#include <opencv2/opencv.hpp>
#include <vector>
#define MARKER_NEW 1
#if MARKER_NEW //new for camera paper
#define MARKER_ROW_NUM 6//hang
#define MARKER_COL_NUM 7//lie
#define MARKERS_ROW_DISTANCE 46.0	//cm
#define MARKERS_COL_DISTANCE 45.0	//cm
#define MARKER_SIZE 20//cm
#else
#define MARKER_ROW_NUM 6
#define MARKER_COL_NUM 6
#define MARKERS_ROW_DISTANCE 60.0	//cm
#define MARKERS_COL_DISTANCE 50.0	//cm
#define MARKER_SIZE 20//cm
#endif

class MarkerConfig
{
public:
	MarkerConfig();
    MarkerConfig(int _id, float _boardsize, float _x, float _y, float _z);
	~MarkerConfig();

	int id;
    float boardsize;
	cv::Point3f coordinate;
private:

};


class MarkerWorld
{
public:
	MarkerWorld();
	MarkerWorld(int _id, cv::Point3f _coordinate);
	MarkerWorld(int _id, float _x, float _y, float _z);
	~MarkerWorld();

	cv::Point3f coordinate;

	int id;

private:

};


class MarkerWorldCoordinate
{
public:
	MarkerWorldCoordinate();
	MarkerWorldCoordinate(size_t _size);
	~MarkerWorldCoordinate();

	size_t size();
	bool setCoordinate(MarkerWorld mw);
	cv::Point3f getCoordinate(int _id);

private:
	size_t m_size;
	std::vector<MarkerWorld> coorTable;
};

void initCoordinateTable(std::vector<MarkerWorld> &CoordinateTable);

#endif // __MarkerWorldCoornidate_h__
