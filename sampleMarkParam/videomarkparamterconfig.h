﻿#ifndef VIDEOMARKPARAMTERCONFIG_H
#define VIDEOMARKPARAMTERCONFIG_H

#include <QString>

typedef enum TrackingMethod{
    UNTRACKINGMETHOD = -1,
    KALMAN = 0,
}TrackingMethod;

class VideoMarkParamterConfig
{
public:
    VideoMarkParamterConfig();
    ~VideoMarkParamterConfig();

    void setSkipFrameNumber(int number);
    void setIsTracking(bool whether);
    void setTrackingMethod(int method);

    static int getSkipFrameNumber();
    static bool getIsTracking();
    static TrackingMethod getTrackingMethod();

    static int loadConfig();
    static int saveConfig();

private:

    static int skipFrameNumber;
    static bool isTracking;
    static TrackingMethod trackingMethod;

private:

    void init();
};

#endif // VIDEOMARKPARAMTERCONFIG_H
