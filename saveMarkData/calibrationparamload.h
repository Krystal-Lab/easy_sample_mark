#ifndef CALIBRATIONPARAMLOAD_H
#define CALIBRATIONPARAMLOAD_H

#include <opencv2/core.hpp>

#include <QObject>

class CalibrationParamLoad : public QObject
{
    Q_OBJECT
public:
    CalibrationParamLoad(QObject *parent = nullptr);
    ~CalibrationParamLoad();

    bool loadCameraIntrinsic(const QString &filePath, cv::Mat &cameraInstrinsics, cv::Mat &distortionCoefficients);
    bool loadCameraHomography(const QString &filePath, cv::Mat &homography);

signals:

};

#endif // CALIBRATIONPARAMLOAD_H
