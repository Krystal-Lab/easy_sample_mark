#include "calibrationparamload.h"
#include <QFileInfo>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QtDebug>

CalibrationParamLoad::CalibrationParamLoad(QObject *parent) : QObject(parent)
{

}

CalibrationParamLoad::~CalibrationParamLoad()
{

}

bool CalibrationParamLoad::loadCameraIntrinsic(const QString &filePath, cv::Mat &cameraInstrinsics, cv::Mat &distortionCoefficients)
{
    bool result = false;
    QByteArray data;
    QFile file;
    file.setFileName(filePath);

    cameraInstrinsics = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    distortionCoefficients = cv::Mat(5, 1,CV_32FC1, cv::Scalar::all(0));

    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        return result;
    }
    data = file.readAll();
    file.close();
    QJsonParseError jsonError;
    QJsonDocument parseDoucment = QJsonDocument::fromJson(QString(data).toUtf8(), &jsonError);

    if(jsonError.error == QJsonParseError::NoError)
    {
        if (!(parseDoucment.isNull() || parseDoucment.isEmpty()))
        {
            if (parseDoucment.isObject())
            {
                QJsonObject jsonObject = parseDoucment.object();
                if(jsonObject.contains("camera_intrinsic") && jsonObject.contains("camera_distortion"))
                {
                    QJsonArray intrinsicList = jsonObject.take("camera_intrinsic").toArray();
                    for(int index = 0; index < intrinsicList.size(); index++)
                    {
                        QJsonArray rowData = intrinsicList.at(index).toArray();
                        cameraInstrinsics.at<float>(index, 0) = static_cast<float>(rowData.at(0).toDouble());
                        cameraInstrinsics.at<float>(index, 1) = static_cast<float>(rowData.at(1).toDouble());
                        cameraInstrinsics.at<float>(index, 2) = static_cast<float>(rowData.at(2).toDouble());
                    }

                    QJsonArray distList = jsonObject.take("camera_distortion").toArray();
                    int count = std::min(5, distList.size());
                    for(int index = 0; index < count; index++)
                    {
                        distortionCoefficients.at<float>(index, 0) = static_cast<float>(distList.at(index).toDouble());
                    }

                    result = true;
                }
            }
        }
    }
    else
    {
        qDebug() << "error:" << jsonError.errorString() << endl;
        return result;
    }
    return result;
}

bool CalibrationParamLoad::loadCameraHomography(const QString &filePath, cv::Mat &homography)
{
    bool result = false;
    QByteArray data;
    QFile file;
    file.setFileName(filePath);

    homography = cv::Mat(3, 3, CV_32F, cv::Scalar::all (0));

    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        return result;
    }
    data = file.readAll();
    file.close();
    QJsonParseError jsonError;
    QJsonDocument parseDoucment = QJsonDocument::fromJson(QString(data).toUtf8(), &jsonError);

    if(jsonError.error == QJsonParseError::NoError)
    {
        if (!(parseDoucment.isNull() || parseDoucment.isEmpty()))
        {
            if (parseDoucment.isObject())
            {
                QJsonObject jsonObject = parseDoucment.object();
                if(jsonObject.contains("camera_homography"))
                {
                    QJsonArray paramObject = jsonObject.take("camera_homography").toArray();
                    int tempIndex = 0;
                    for(int r = 0; r < homography.rows; r++)
                    {
                        for(int c = 0; c < homography.cols; c++)
                        {
                            homography.at<float>(r, c) = static_cast<float>(paramObject.at(tempIndex++).toDouble());
                        }
                    }
                    result = true;
                }
            }
        }
    }
    else
    {
        qDebug() << "error:" << jsonError.errorString() << endl;
        return result;
    }
    return result;
}

