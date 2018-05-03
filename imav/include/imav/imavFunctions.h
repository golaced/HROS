#ifndef __IMAV_FUNCTIONS_H__
#define __IMAV_FUNCTIONS_H__

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <imav/Barrel.h>
#include <imav/BarrelList.h>
#include <sensor_msgs/NavSatFix.h>

#define MISSION_IDLE 0
#define MISSION_CHECK_BARREL 1
#define MISSION_SAVE_GPS 2

#define RIGHT_ANGLE 90.0
#define IMAV_PI 3.1415926535898
#define EARTH_RADIUS 6378.137

#define OUT_AREA_THREAD 10 //m

static std::string lonlat2degreefenmiao(double lonlat)
{
    double degree = double(int(lonlat));
    double fen = double(lonlat - degree) * 60;
    double miao = (fen - (int)fen) * 60;
    fen = int(fen);

    if (fabs(miao) < 0.00000001)
    {
        miao = 0.0;
    }

    std::stringstream ss;
    ss << degree << "°" << fen << "'" << miao << "''";
    std::string result;
    ss >> result;

    return result;
}

/**
 * @brief get rad according to the degrees. 
 * @param degree: 
 * 
 * @return: rad according to the degrees. 
 */
static double Deg2Rad(double degree)
{
    return degree * IMAV_PI / (RIGHT_ANGLE * 2);
}

/**
 * @brief calc the direction distance of two coordinates.
 * @param srcLon: the longitude of source coordinate;
 *        srcLat: the latitude of source coordinate;
 *        destLon: the longitude of destination coordinate;
 *        destLat: the latitude of destination coordinate;
 * 
 * @return DirectDistance: the direction distance from source coordinate to destination coordinate.
 */
static double GetDirectDistance(double srcLat, double srcLon, double destLat, double destLon)
{
    double radSrcLat = Deg2Rad(srcLat);
    double radDestLat = Deg2Rad(destLat);
    double a = radSrcLat - radDestLat;
    double b = Deg2Rad(srcLon) - Deg2Rad(destLon);

    double DirectDistance = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radSrcLat) * cos(radDestLat) * pow(sin(b / 2), 2)));

    DirectDistance = DirectDistance * EARTH_RADIUS;
    DirectDistance = round(DirectDistance * 10000) / 10000 * 1000;

    return DirectDistance;
}

static bool outofcheckedarea(sensor_msgs::NavSatFix current_gps, imav::BarrelList checkedbarrels)
{
    for (size_t i = 0; i < checkedbarrels.barrels.size(); i++)
    {
        if (GetDirectDistance(current_gps.latitude, current_gps.longitude, checkedbarrels.barrels[i].latitude, checkedbarrels.barrels[i].longitude) < OUT_AREA_THREAD)
        {
            return 0;
        }
    }
    return 1;
}

static bool sortBarrelsByArea1(const imav::Barrel &b1, const imav::Barrel &b2)
{
    return b1.area > b2.area;
}

static void sortBarrelsByArea(imav::BarrelList &barrellist)
{
    std::sort(barrellist.barrels.begin(), barrellist.barrels.end(), sortBarrelsByArea1);
}

inline std::string expand_user(std::string path)
{
    if (not path.empty() and path[0] == '~')
    {
        assert(path.size() == 1 or path[1] == '/'); // or other error handling
        char const *home = getenv("HOME");
        if (home or ((home = getenv("USERPROFILE"))))
        {
            path.replace(0, 1, home);
        }
        else
        {
            char const *hdrive = getenv("HOMEDRIVE"),
                       *hpath = getenv("HOMEPATH");
            assert(hdrive); // or other error handling
            assert(hpath);
            path.replace(0, 1, std::string(hdrive) + hpath);
        }
    }
    return path;
}

static std::string initBarrelWritePath()
{
    using std::cout;
    using std::endl;
#ifdef WIN32

#else
    std::string home_path = expand_user("~");
    home_path += "/";
    cout << "home_path:" << home_path << endl;

    std::string barrel_num_file_path = home_path;
    std::string writer_path = home_path;
    barrel_num_file_path += "workspace/imav/barrel_file_num.txt";
    writer_path += "workspace/imav/";
#endif // WIN32

    cout << "barrel_file_num path:" << barrel_num_file_path << endl;
    cout << "barrel_gps writer_path:" << writer_path << endl;

    int barrel_file_num = 0;

    std::ifstream barrel_num_file_read;
    barrel_num_file_read.open(barrel_num_file_path.c_str());
    barrel_num_file_read >> barrel_file_num;
    barrel_num_file_read.close();

    cout << "barrel_file_num :" << barrel_file_num << endl;

    std::ofstream barrel_num_file_writer;
    barrel_num_file_writer.open(barrel_num_file_path.c_str());
    barrel_num_file_writer << (barrel_file_num + 1);
    barrel_num_file_writer.close();

    std::stringstream ss;
    std::string barrel_gps_file_name;

    ss << barrel_file_num;
    ss >> barrel_gps_file_name;
    barrel_gps_file_name += ".kml";
    std::ofstream barrel_gps_writer;
    barrel_gps_writer.open((writer_path + barrel_gps_file_name).c_str(), std::ios::app);
    barrel_gps_writer << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
	barrel_gps_writer << "<kml>" << endl;
    barrel_gps_writer << "<Folder>" << endl;
    barrel_gps_writer.close();
    return (writer_path + barrel_gps_file_name);
}

#endif //__IMAV_FUNCTIONS_H__
