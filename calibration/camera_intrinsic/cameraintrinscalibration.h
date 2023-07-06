#ifndef CAMERAINTRINSCALIBRATION_H
#define CAMERAINTRINSCALIBRATION_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>

#define MIN_CALI_IMAGE_NUM (15)

class CameraIntrinsCalibration
{
public:
    CameraIntrinsCalibration();
    ~CameraIntrinsCalibration();

    void setInitData(const int camera_model, const cv::Size &image_size, const cv::Size &board_size,
                     const cv::Size &square_size, const std::string &save_result_dir);

    void setCameraParam(const int camera_model, const cv::Mat& intrinsic, const cv::Mat &distortion);

    bool calibrating(const std::vector<std::string> &images_list, std::string &err_result);

    int getSuccessImage();
    void saveCalibrationResult();

    cv::Mat getUndistortImage(const std::string &image_path);
    cv::Mat getDrawCornerImage(const std::string &image_path, const int index);

    void saveDrawCornerImage(const std::vector<std::string> &images_list);
    void saveUndistortImage(const std::vector<std::string> &images_list);

    void getIntrinsicParam(cv::Matx33d &intrinsic, std::vector<double> &distortion);

private:
    void getCorners(const std::vector<std::string> &images_list);
    void getCornersCoordinate(std::vector<std::vector<cv::Point3f>> &object_points,
                              std::vector<int> &point_counts,
                              std::vector<std::vector<cv::Point2f>> &use_image_corners);
    std::string  calibrationEvaluate(const std::vector<cv::Vec3d> &rotation_vectors,
                                     const std::vector<cv::Vec3d> &translation_vectors,
                                     const std::vector<std::vector<cv::Point3f>> &object_points,
                                     const std::vector<int> &point_counts,
                                     const std::vector<std::vector<cv::Point2f>> &use_image_corners);

private:
    cv::Size image_size;
    cv::Size board_size;
    cv::Size square_size;
    cv::Matx33d intrinsic_matrix;
    cv::Vec4d distortion_coeffs;
    cv::Mat pinhole_distortion_coeffs;
    std::vector<cv::Vec3d> rotation_vectors;                           /* 每幅图像的旋转向量 */
    std::vector<cv::Vec3d> translation_vectors;                        /* 每幅图像的平移向量 */
    std::vector<double> error_list;
    std::vector<std::vector<cv::Point2f>> corners_Seq;
    std::vector<bool> is_use;

    int camera_model;

    std::string saveResultPath;
    std::string saveResultDir;
};

#endif // CAMERAINTRINSCALIBRATION_H
