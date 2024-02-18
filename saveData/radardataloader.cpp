#include "radardataloader.h"
#include <iostream>
#include <fstream>

RadarDataLoader::RadarDataLoader(QObject *parent) : QObject(parent)
{

}

RadarDataLoader::~RadarDataLoader()
{

}

bool RadarDataLoader::loadRadarData(const QString &filePath, std::vector<cv::Point3f> &point3DList)
{
    point3DList.clear();
    std::ifstream file(filePath.toStdString());
    if (!file.is_open())
    {
        std::cout << "ERROR--->>> cannot open: " << filePath.toStdString() << std::endl;
        return false;
    }
    std::string line;
    getline(file, line);
    // conti radar: the input point is the x and y coordinate
    bool whether_first = true;
    std::string first_time_str;
    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string str;
        std::string time_str;
        std::string position_x_str;
        std::string position_y_str;
        int index = 0;
        while (getline(ss, str, ','))
        {
            if (index == 0) {
                time_str = str;
                if (whether_first) {
                    first_time_str = str;
                    whether_first = false;
                } else {
                    long long gap = std::stoll(time_str) - std::stoll(first_time_str);
                    if (gap > 15 * 1e6) {
                        std::cout << "radar point size: " << point3DList.size() << std::endl;
                        return true;
                    }
                }
            }
            if (index == 4) {
                position_x_str = str;
            } else if (index == 5) {
                position_y_str = str;
            }
            index++;
        }

        cv::Point3f radar_point;
        radar_point.x = std::atof(position_x_str.c_str());
        radar_point.y = std::atof(position_y_str.c_str());
        radar_point.z = 0;
        if (std::abs(radar_point.x) < 1e-6 || std::abs(radar_point.y) < 1e-6)
        {
            continue;
        }
        point3DList.push_back(radar_point);
    }

    if(point3DList.size() > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

