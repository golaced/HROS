#include "MarkerWorldCoornidate.h"

std::vector<MarkerWorld> CoordinateTable;


//MarkerWorld CoordinateTable[100]; = {
//	MarkerWorld(1, cv::Point3f(0 * MARKERS_COL_DISTANCE, 0 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(2, cv::Point3f(1 * MARKERS_COL_DISTANCE, 0 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(3, cv::Point3f(2 * MARKERS_COL_DISTANCE, 0 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(4, cv::Point3f(3 * MARKERS_COL_DISTANCE, 0 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(5, cv::Point3f(4 * MARKERS_COL_DISTANCE, 0 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(6, cv::Point3f(5 * MARKERS_COL_DISTANCE, 0 * MARKERS_ROW_DISTANCE, 0)),
//
//	MarkerWorld(7, cv::Point3f(0 * MARKERS_COL_DISTANCE, 1 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(8, cv::Point3f(1 * MARKERS_COL_DISTANCE, 1 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(9, cv::Point3f(2 * MARKERS_COL_DISTANCE, 1 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(10, cv::Point3f(3 * MARKERS_COL_DISTANCE, 1 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(11, cv::Point3f(4 * MARKERS_COL_DISTANCE, 1 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(12, cv::Point3f(5 * MARKERS_COL_DISTANCE, 1 * MARKERS_ROW_DISTANCE, 0)),
//
//	MarkerWorld(13, cv::Point3f(0 * MARKERS_COL_DISTANCE, 2 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(14, cv::Point3f(1 * MARKERS_COL_DISTANCE, 2 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(15, cv::Point3f(2 * MARKERS_COL_DISTANCE, 2 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(16, cv::Point3f(3 * MARKERS_COL_DISTANCE, 2 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(17, cv::Point3f(4 * MARKERS_COL_DISTANCE, 2 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(18, cv::Point3f(5 * MARKERS_COL_DISTANCE, 2 * MARKERS_ROW_DISTANCE, 0)),
//
//	MarkerWorld(19, cv::Point3f(0 * MARKERS_COL_DISTANCE, 3 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(20, cv::Point3f(1 * MARKERS_COL_DISTANCE, 3 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(21, cv::Point3f(2 * MARKERS_COL_DISTANCE, 3 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(22, cv::Point3f(3 * MARKERS_COL_DISTANCE, 3 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(23, cv::Point3f(4 * MARKERS_COL_DISTANCE, 3 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(24, cv::Point3f(5 * MARKERS_COL_DISTANCE, 3 * MARKERS_ROW_DISTANCE, 0)),
//
//	MarkerWorld(25, cv::Point3f(0 * MARKERS_COL_DISTANCE, 4 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(26, cv::Point3f(1 * MARKERS_COL_DISTANCE, 4 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(27, cv::Point3f(2 * MARKERS_COL_DISTANCE, 4 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(28, cv::Point3f(3 * MARKERS_COL_DISTANCE, 4 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(29, cv::Point3f(4 * MARKERS_COL_DISTANCE, 4 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(30, cv::Point3f(5 * MARKERS_COL_DISTANCE, 4 * MARKERS_ROW_DISTANCE, 0)),
//
//	MarkerWorld(31, cv::Point3f(0 * MARKERS_COL_DISTANCE, 5 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(32, cv::Point3f(1 * MARKERS_COL_DISTANCE, 5 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(33, cv::Point3f(2 * MARKERS_COL_DISTANCE, 5 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(34, cv::Point3f(3 * MARKERS_COL_DISTANCE, 5 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(35, cv::Point3f(4 * MARKERS_COL_DISTANCE, 5 * MARKERS_ROW_DISTANCE, 0)),
//	MarkerWorld(36, cv::Point3f(5 * MARKERS_COL_DISTANCE, 5 * MARKERS_ROW_DISTANCE, 0))
//
//};


MarkerConfig::MarkerConfig()
{
	id = -1;
	boardsize = -1;
	coordinate = cv::Point3f(-1,-1,-1);
}

MarkerConfig::MarkerConfig(int _id, float _boardsize, float _x, float _y, float _z)
{
	id = _id;
	boardsize = _boardsize;
	coordinate = cv::Point3f(_x,_y,_z);
}

MarkerConfig::~MarkerConfig()
{
}

void initCoordinateTable(std::vector<MarkerWorld> &CoordinateTable)
{


	for(size_t id=0;id<100;id++)
	{
		int x = (id%MARKER_COL_NUM-1+(id%MARKER_COL_NUM==0?MARKER_COL_NUM:0))*MARKERS_COL_DISTANCE;
		int y=(id/MARKER_COL_NUM-(id%MARKER_COL_NUM==0?1:0))*MARKERS_ROW_DISTANCE;
		MarkerWorld mw(id, x,y,0);
		CoordinateTable.push_back(mw);
	}
	//CoordinateTable.push
}

MarkerWorld::MarkerWorld()
{
	coordinate = cv::Point3f(0.0, 0.0, 0.0);
	id = -1;
}

MarkerWorld::MarkerWorld(int _id, float _x, float _y, float _z)
{
	coordinate = cv::Point3f(_x, _y, _z);
	id = _id;
}

MarkerWorld::MarkerWorld(int _id, cv::Point3f _coordinate)
{
	coordinate = cv::Point3f(_coordinate.x, _coordinate.y, _coordinate.z);
	id = _id;
}

MarkerWorld::~MarkerWorld()
{

}

MarkerWorldCoordinate::MarkerWorldCoordinate(size_t _size)
{
	m_size = _size;
	coorTable.resize(m_size, MarkerWorld(0, cv::Point3f(0, 0, 0)));

	for (size_t i = 0; i < m_size; i++)
	{
		coorTable[i].id = i + 1;
	}

}

MarkerWorldCoordinate::MarkerWorldCoordinate()
{
	m_size = 0;
	coorTable.resize(m_size);
}

size_t MarkerWorldCoordinate::size()
{
	return m_size;
}

bool MarkerWorldCoordinate::setCoordinate(MarkerWorld mw)
{
	coorTable[mw.id].coordinate = cv::Point3f(mw.coordinate.x, mw.coordinate.y, mw.coordinate.z);

	return true;
}

cv::Point3f MarkerWorldCoordinate::getCoordinate(int _id)
{
	return coorTable[_id].coordinate;
}

MarkerWorldCoordinate::~MarkerWorldCoordinate()
{

}
