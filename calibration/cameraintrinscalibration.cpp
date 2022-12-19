#include "cameraintrinscalibration.h"
#include <sstream>

CameraIntrinsCalibration::CameraIntrinsCalibration()
{
    image_size = cv::Size(1920, 1020);
    board_size = cv::Size(11, 8);
    square_size = cv::Size(20, 20);//mm
    is_use.clear();
    rotation_vectors.clear();
    translation_vectors.clear();
    corners_Seq.clear();
    camera_model = 1;

    intrinsic_matrix = cv::Matx33f::zeros();
//    distortion_coeffs = cv::Vec4d(0.0, 0.0, 0.0, 0.0);

    pinhole_distortion_coeffs = cv::Mat(1,5,CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
}

CameraIntrinsCalibration::~CameraIntrinsCalibration()
{

}

void CameraIntrinsCalibration::setInitData(const int camera_model, const cv::Size &image_size, const cv::Size &board_size,
                                           const cv::Size &square_size, const std::string &save_result_dir)
{
    this->camera_model = camera_model;
    this->image_size = image_size;
    this->board_size = board_size;
    this->square_size = square_size;
    this->saveResultDir = save_result_dir;
    this->saveResultPath = save_result_dir + "/" + "calibration_result.txt";
    is_use.clear();
    rotation_vectors.clear();
    translation_vectors.clear();
    corners_Seq.clear();
}

bool CameraIntrinsCalibration::calibrating(const std::vector<std::string> &images_list, std::string &err_result)
{
    bool success = false;
    std::vector<std::vector<cv::Point2f>> use_image_corners;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<int> point_counts;
    int flags = 0;

    is_use.clear();
    rotation_vectors.clear();
    translation_vectors.clear();
    corners_Seq.clear();

    getCorners(images_list);
    getCornersCoordinate(object_points, point_counts, use_image_corners);

    if(is_use.size() > 1)
    {
        std::cout << "temp:" << object_points[0].size() << " " << point_counts[0] << " " << use_image_corners[0].size() << std::endl;
        if(this->camera_model == 1)
        {
            cv::calibrateCamera(object_points, use_image_corners, image_size,
                                intrinsic_matrix, pinhole_distortion_coeffs,
                                rotation_vectors, translation_vectors, cv::CALIB_FIX_K3,
                                cv::TermCriteria(3, 20, 1e-6));
        }
        else if(this->camera_model == 2)
        {
            flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
            flags |= cv::fisheye::CALIB_CHECK_COND;
            flags |= cv::fisheye::CALIB_FIX_SKEW;
            cv::fisheye::calibrate(object_points, use_image_corners, image_size, intrinsic_matrix, distortion_coeffs,
                               rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
        }
        err_result = calibrationEvaluate(rotation_vectors, translation_vectors, object_points, point_counts, use_image_corners);
        success = true;
    }
    return success;
}

int CameraIntrinsCalibration::getSuccessImage()
{
    int result = 0;
    for (size_t i = 0; i < is_use.size(); i++)
    {
        if(is_use[i])
        {
            result++;
        }
    }
    return result;
}

void CameraIntrinsCalibration::saveCalibrationResult()
{
    std::ofstream fout(saveResultPath);
    cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    int successImageNum = 0;

    fout << "相机内参数矩阵：" << std::endl;
    fout << intrinsic_matrix << std::endl;
    fout << "畸变系数：\n";
    if(this->camera_model == 1)
    {
        fout << pinhole_distortion_coeffs << std::endl;
    }
    else if(this->camera_model == 2)
    {
        fout << distortion_coeffs << std::endl;
    }
    for(size_t i = 0; i < is_use.size(); i++)
    {
        if(is_use[i])
        {
            fout << "第" << i + 1 << "幅图像的旋转向量：" << std::endl;
            fout << rotation_vectors[successImageNum] << std::endl;

            /* 将旋转向量转换为相对应的旋转矩阵 */
            cv::Rodrigues(rotation_vectors[successImageNum], rotation_matrix);
            fout << "第" << i + 1 << "幅图像的旋转矩阵：" << std::endl;
            fout << rotation_matrix << std::endl;
            fout << "第" << i + 1 << "幅图像的平移向量：" << std::endl;
            fout << translation_vectors[successImageNum] << std::endl;
            successImageNum++;
        }
    }
    fout << std::endl;
}

cv::Mat CameraIntrinsCalibration::getUndistortImage(const std::string &image_path)
{
    cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
    cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat image = cv::imread(image_path);
    cv::Mat result = image.clone();
    if(this->camera_model == 1)
    {
//        cv::initUndistortRectifyMap(intrinsic_matrix, pinhole_distortion_coeffs,
//                                    R, intrinsic_matrix,image_size, CV_32FC1, mapx, mapy);
//        cv::remap(image, result, mapx, mapy, cv::INTER_LINEAR);
        cv::undistort(image, result, intrinsic_matrix, pinhole_distortion_coeffs);
    }
    else if(this->camera_model == 2)
    {
        //cv::fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
        cv::fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R,
                                         cv::getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0),
                                         image_size, CV_32FC1, mapx, mapy);
        cv::remap(image, result, mapx, mapy, cv::INTER_LINEAR);
    }
    return result;
}

cv::Mat CameraIntrinsCalibration::getDrawCornerImage(const std::string &image_path, const int index)
{
    cv::Mat image;
    if(corners_Seq.size() > 0)
    {
        if(is_use[index])
        {
            image = cv::imread(image_path);
            std::vector<cv::Point2f> corners = corners_Seq[index];
            cv::drawChessboardCorners(image, board_size, corners, true);
        }
    }
    return image;
}

void CameraIntrinsCalibration::saveDrawCornerImage(const std::vector<std::string> &images_list)
{
    for (size_t i = 0; i < is_use.size(); i++)
    {
        if(is_use[i])
        {
            cv::Mat image = getDrawCornerImage(images_list[i], i);
            std::string imageFileName;
            std::stringstream StrStm;
            StrStm << saveResultDir << "/"<< i + 1;
            StrStm >> imageFileName;
            imageFileName += "_corner.jpg";
            cv::imwrite(imageFileName, image);
        }
    }
}

void CameraIntrinsCalibration::saveUndistortImage(const std::vector<std::string> &images_list)
{
    cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
    cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    for (size_t i = 0; i < images_list.size(); i++)
    {
        std::cout << "Frame #" << i + 1 << "..." << std::endl;
        cv::Mat result = getUndistortImage(images_list[i]);
        std::string imageFileName;
        std::stringstream StrStm;
        StrStm << saveResultDir << "/"<< i + 1;
        StrStm >> imageFileName;
        imageFileName += "_d.png";
        cv::imwrite(imageFileName, result);
    }
}

void CameraIntrinsCalibration::getCorners(const std::vector<std::string> &images_list)
{
    int image_count = images_list.size();
    int count = 0;
    corners_Seq.clear();
    is_use.clear();
    for (int i = 0; i < image_count; i++)
    {
        std::cout << "Frame #" << i + 1 << "..." << std::endl;
        std::vector<cv::Point2f> corners;
        cv::Mat image = cv::imread(images_list[i]);
        /* 提取角点 */
        cv::Mat imageGray;
        cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
        bool patternfound = cv::findChessboardCorners(image, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH +
                                                      cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        if (!patternfound)
        {
            corners.clear();
            corners_Seq.push_back(corners);
            is_use.push_back(false);
            std::cout << "找不到角点，需删除图片文件" << images_list[i] << "重新排列文件名，再次标定" << std::endl;
        }
        else
        {
            /* 亚像素精确化 */
//            cv::cornerSubPix(imageGray, corners, cv::Size(11, 11), cv::Size(-1, -1),
//                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            cv::find4QuadCornerSubpix(imageGray, corners, cv::Size(11, 11));
            count = count + corners.size();
            corners_Seq.push_back(corners);
            is_use.push_back(true);
            std::cout << "Frame corner#" << i + 1 << "...end" << std::endl;
        }
    }
}

void CameraIntrinsCalibration::getCornersCoordinate(std::vector<std::vector<cv::Point3f>> &object_points,
                                              std::vector<int> &point_counts,
                                              std::vector<std::vector<cv::Point2f>> &use_image_corners)
{
    object_points.clear();
    point_counts.clear();
    use_image_corners.clear();
    /* 初始化定标板上角点的三维坐标 */
    for (size_t t = 0; t< is_use.size(); t++)
    {
        if(is_use[t])
        {
            std::vector<cv::Point3f> tempPointSet;
            for (int i = 0; i<board_size.height; i++)
            {
                for (int j = 0; j<board_size.width; j++)
                {
                    /* 假设定标板放在世界坐标系中z=0的平面上 */
                    cv::Point3f tempPoint;
                    tempPoint.x = i*square_size.width;
                    tempPoint.y = j*square_size.height;
                    tempPoint.z = 0;
                    tempPointSet.push_back(tempPoint);
                }
            }
            object_points.push_back(tempPointSet);
        }
    }
    for (size_t i = 0; i< is_use.size(); i++)
    {
        if(is_use[i])
        {
            point_counts.push_back(board_size.width*board_size.height);
            use_image_corners.push_back(corners_Seq[i]);
        }
    }
}

std::string  CameraIntrinsCalibration::calibrationEvaluate(const std::vector<cv::Vec3d> &rotation_vectors,
                                                           const std::vector<cv::Vec3d> &translation_vectors,
                                                           const std::vector<std::vector<cv::Point3f>> &object_points,
                                                           const std::vector<int> &point_counts,
                                                           const std::vector<std::vector<cv::Point2f>> &use_image_corners)
{
    std::ostringstream result;
    double total_err = 0.0;
    double err = 0.0;
    int successImageNum = 0;
    std::vector<cv::Point2f>  image_points2;
    for(size_t i = 0; i < is_use.size(); i++)
    {
        if(is_use[i])
        {
            std::vector<cv::Point3f> tempPointSet = object_points[successImageNum];
            /****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
            if(this->camera_model == 1)
            {
                cv::projectPoints(tempPointSet,
                                  rotation_vectors[successImageNum],
                                  translation_vectors[successImageNum],
                                  intrinsic_matrix, pinhole_distortion_coeffs, image_points2);
            }
            else if(this->camera_model == 2)
            {
                cv::fisheye::projectPoints(tempPointSet, image_points2,
                                       rotation_vectors[successImageNum],
                                       translation_vectors[successImageNum],
                                       intrinsic_matrix, distortion_coeffs);
            }
            /* 计算新的投影点和旧的投影点之间的误差*/
            std::vector<cv::Point2f> tempImagePoint = use_image_corners[successImageNum];
            cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
            cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
            for (size_t i = 0; i != tempImagePoint.size(); i++)
            {
                image_points2Mat.at<cv::Vec2f>(0, i) = cv::Vec2f(image_points2[i].x, image_points2[i].y);
                tempImagePointMat.at<cv::Vec2f>(0, i) = cv::Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
            }
            err = cv::norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
            total_err += err /= point_counts[successImageNum];
            result << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << std::endl;
            successImageNum++;
        }
    }
    if(successImageNum > 0)
    {
        result << "总体平均误差：" << total_err / successImageNum << "像素" << std::endl;
    }
    result << "相机内参数矩阵：" << std::endl;
    result << intrinsic_matrix << std::endl;
    result << "畸变系数：\n";
    if(this->camera_model == 1)
    {
        result << pinhole_distortion_coeffs << std::endl;
    }
    else if(this->camera_model == 2)
    {
        result << distortion_coeffs << std::endl;
    }
    return result.str();
}
