#ifndef RADARDATALOADER_H
#define RADARDATALOADER_H

#include <opencv2/core.hpp>

#include <QObject>

class RadarDataLoader : public QObject
{
    Q_OBJECT
public:
    RadarDataLoader(QObject *parent = nullptr);
    ~RadarDataLoader();

    bool loadRadarData(const QString &filePath, std::vector<cv::Point3f> &point3DList);

};

#endif // RADARDATALOADER_H
