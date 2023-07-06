#ifndef FISHEYECAMERAUNDISTORTION_H
#define FISHEYECAMERAUNDISTORTION_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

class FisheyeCameraUndistortion
{
public:
    FisheyeCameraUndistortion();
    ~FisheyeCameraUndistortion();

    cv::Mat transverseCorrection(const cv::Mat &input_image);

    cv::Mat longitudeCorrection(const cv::Mat &input_image);

    cv::Mat latitudeCorrection(const cv::Mat &input_image);

    cv::Mat dewarpCorrection(const cv::Mat &input_image);

    cv::Mat getUsefulArea(const cv::Mat &input_image);

private:
    void rectifyMap(cv::Mat &mapImg, const int inWidth, const int inHeight,
                    const float* rot, const int outWidth, const int outHeight,
                    const float FOV, const float radius);
    void remap(const cv::Mat& srcImg, cv::Mat& dstImg, const cv::Mat& mapImg,
               int inHeight, int inWidth, int outHeight, int outWidth);
};

#endif // FISHEYECAMERAUNDISTORTION_H
