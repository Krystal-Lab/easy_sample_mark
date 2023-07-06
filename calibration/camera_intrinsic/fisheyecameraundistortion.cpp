#include "fisheyecameraundistortion.h"
#include <algorithm>
#include <iostream>

const static float PI = 3.1415926f;

static bool compareInterval(const std::vector<cv::Point> &one, const std::vector<cv::Point> &two)
{
    double temp1 = cv::contourArea(one);
    double temp2 = cv::contourArea(two);
    return (temp1 > temp2);
}

FisheyeCameraUndistortion::FisheyeCameraUndistortion()
{

}

FisheyeCameraUndistortion::~FisheyeCameraUndistortion()
{

}

cv::Mat FisheyeCameraUndistortion::getUsefulArea(const cv::Mat &input_image)
{
    cv::Mat image_gray;
    cv::Mat image_binary;
    std::vector< std::vector<cv::Point> > contours;
    cv::Point2f center;
    float radius;
    cv::Mat mask = cv::Mat::zeros(input_image.rows, input_image.cols, input_image.type());
    cv::cvtColor(input_image, image_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(image_gray, image_binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::findContours(image_binary, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours.begin(), contours.end(), compareInterval);
    cv::minEnclosingCircle(contours[0], center, radius);
    cv::circle(mask, center, radius, cv::Scalar(1, 1, 1), -1);
    cv::Mat temp_image = input_image * mask;
    cv::Mat result = temp_image(cv::Rect(int(center.x) - int(radius), int(center.y) - int(radius),
                                         int(center.x) + int(radius), int(center.y) + int(radius))).clone();
    return result;
}

cv::Mat FisheyeCameraUndistortion::transverseCorrection(const cv::Mat &input_image)
{
    cv::Mat result = input_image.clone();
    int nbottom = 0;
    int ntop = 0;
    int nright = 0;
    int nleft = 0;

    //遍历寻找上边界
    int nflag = 0;
    for (int i=0 ;i< input_image.rows -1;i++)
    {
        for (int j=0; j< input_image.cols -1; j++)
        {
            uchar I = 0.59*input_image.at<cv::Vec3b>(i,j)[0] + 0.11*input_image.at<cv::Vec3b>(i,j)[1] + 0.3*input_image.at<cv::Vec3b>(i,j)[2];
            if (I > 20)
            {
                I = 0.59*input_image.at<cv::Vec3b>(i+1,j)[0] + 0.11*input_image.at<cv::Vec3b>(i+1,j)[1] + 0.3*input_image.at<cv::Vec3b>(i+1,j)[2];
                if (I > 20)
                {
                    ntop = i;
                    nflag = 1;
                    break;
                }
            }
        }
        if (nflag ==1)
        {
            break;
        }
    }
    //遍历寻找下边界
    nflag = 0;
    for (int i= input_image.rows-1;i > 1;i--)
    {
        for (int j=0; j < input_image.cols -1; j++)
        {
            uchar I = 0.59*input_image.at<cv::Vec3b>(i,j)[0] + 0.11*input_image.at<cv::Vec3b>(i,j)[1] + 0.3*input_image.at<cv::Vec3b>(i,j)[2];
            if (I > 20)
            {
                I = 0.59*input_image.at<cv::Vec3b>(i-1,j)[0] + 0.11*input_image.at<cv::Vec3b>(i-1,j)[1] + 0.3*input_image.at<cv::Vec3b>(i-1,j)[2];
                if (I > 20)
                {
                    nbottom = i;
                    nflag = 1;
                    break;
                }
            }
        }
        if (nflag ==1)
        {
            break;
        }
    }
    //遍历寻找左边界
    nflag = 0;
    for (int j=0; j<input_image.cols -1; j++)
    {
        for (int i=0 ;i< input_image.rows ;i++)
        {
            uchar I = 0.59*input_image.at<cv::Vec3b>(i,j)[0] + 0.11*input_image.at<cv::Vec3b>(i,j)[1] + 0.3*input_image.at<cv::Vec3b>(i,j)[2];
            if (I > 20)
            {
                I = 0.59*input_image.at<cv::Vec3b>(i,j+1)[0] + 0.11*input_image.at<cv::Vec3b>(i,j+1)[1] + 0.3*input_image.at<cv::Vec3b>(i,j+1)[2];
                if (I > 20)
                {
                    nleft = j;
                    nflag = 1;
                    break;
                }
            }
        }
        if (nflag ==1)
        {
            break;
        }
    }
    //遍历寻找右边界
    nflag = 0;
    for (int j=input_image.cols -1; j >0; j--)
    {
        for (int i= 0;i <input_image.rows ;i++)
        {
            uchar I = 0.59*input_image.at<cv::Vec3b>(i,j)[0] + 0.11*input_image.at<cv::Vec3b>(i,j)[1] + 0.3*input_image.at<cv::Vec3b>(i,j)[2];
            if (I > 20)
            {
                I = 0.59*input_image.at<cv::Vec3b>(i,j-1)[0] + 0.11*input_image.at<cv::Vec3b>(i,j-1)[1] + 0.3*input_image.at<cv::Vec3b>(i,j-1)[2];
                if (I > 20)
                {
                    nright = j;
                    nflag = 1;
                    break;
                }
            }
        }
        if (nflag ==1)
        {
            break;
        }
    }
    std::cout << ntop<< std::endl;
    std::cout << nbottom<< std::endl;
    std::cout << nleft << std::endl;
    std::cout << nright << std::endl;

    //根据边界值来获得直径
    int d = cv::min(nright - nleft, nbottom - ntop);

    cv::Mat imgRoi = input_image(cv::Rect(nleft, ntop, d, d)).clone();
    cv::Mat dst(imgRoi.size(), CV_8UC3, cv::Scalar(255,255,255));
    // cv::imwrite("aa.png", imgRoi);

    //建立映射表
    cv::Mat map_x = cv::Mat::zeros(imgRoi.size(), CV_32FC1);
    cv::Mat map_y = cv::Mat::zeros(imgRoi.size(), CV_32FC1);
    for (int j=0; j< d-1;j++)
    {
        for (int i=0; i< d-1; i++ )
        {
            map_x.at<float>(i, j) = static_cast<float>( d/2.0 + i/2.0*cos(1.0*j/d*2*CV_PI));
            map_y.at<float>(i, j) = static_cast<float>( d/2.0 + i/2.0*sin(1.0*j/d*2*CV_PI));
        }
    }
    cv::remap(imgRoi, dst, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
    //重设大小
    cv::resize(dst, result, cv::Size(), 2.0, 1.0);

    // cv::Mat image = getUsefulArea(input_image);
    // cv::imwrite("li.png", image);
//    cv::Mat image = input_image.clone();
//    int d = image.rows / 2;
//    int w = int(2 * CV_PI * d);
//    int h = d;
//    cv::Mat dst = cv::Mat::zeros(h, w, CV_8UC3);
//    cv::Mat map_x = cv::Mat::zeros(h, w, CV_32FC1);
//    cv::Mat map_y = cv::Mat::zeros(h, w, CV_32FC1);
//    for (int i = 0; i < h; i++)
//    {
//        for (int j = 0; j< w; j++)
//        {
//            float angle = static_cast<float>(j / w * CV_PI * 2);
//            int radius = h - i;
//            map_x.at<float>(i, j) = static_cast<float>(d + std::sin(angle) * radius);
//            map_y.at<float>(i, j) = static_cast<float>(d - std::cos(angle) * radius);
//        }
//    }
//    cv::remap(image, result, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

    // cv::imwrite("li.png", result);
    return result;
}

cv::Mat FisheyeCameraUndistortion::longitudeCorrection(const cv::Mat &input_image)
{
    cv::Mat result;
    cv::Mat image = input_image.clone();
    int R = image.rows / 2;
    cv::Mat map_x = cv::Mat::zeros(2 * R, 2 * R, CV_32FC1);
    cv::Mat map_y = cv::Mat::zeros(2 * R, 2 * R, CV_32FC1);
    for (int i = 0; i < map_x.rows; i++)
    {
        for (int j = 0; j< map_x.cols; j++)
        {
            map_x.at<float>(i, j) = static_cast<float>(std::sqrt((j - R) / R * (R * R - (i - R) * (i - R))) + R);
            map_y.at<float>(i, j) = i;
        }
    }
    cv::remap(image, result, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

    cv::imwrite("li.png", result);
    return result;
}

cv::Mat FisheyeCameraUndistortion::latitudeCorrection(const cv::Mat &input_image)
{
    cv::Mat result;
    cv::Mat image = input_image.clone();
    int R = image.rows / 2;
    cv::Mat map_x = cv::Mat::zeros(2 * R, 2 * R, CV_32FC1);
    cv::Mat map_y = cv::Mat::zeros(2 * R, 2 * R, CV_32FC1);
    for (int i = 0; i < map_x.rows; i++)
    {
        for (int j = 0; j< map_x.cols; j++)
        {
            map_x.at<float>(i, j) = i;
            map_y.at<float>(i, j) = static_cast<float>((i - R) / R * std::sqrt((R * R - (j - R) * (j - R))) + R);
        }
    }
    cv::remap(image, result, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

    cv::imwrite("li.png", result);
    return result;
}

cv::Mat FisheyeCameraUndistortion::dewarpCorrection(const cv::Mat &input_image)
{
    //输入鱼眼图像尺寸
    int inHeight = input_image.rows;
    int inWidth = input_image.cols;
    std::cout << "inHeight:" << inHeight << " inWidth:" << inWidth << std::endl;
    //输出经纬度图像尺寸
    int outHeight = inHeight;
    int outWidth = inWidth / 2;
    //视场角
    float FOV = 180;
    //鱼眼半径
    float radius = inHeight / 2.0;

    //以图像中心为赤道
    float rot[9] = {1,0,0,0,1,0,0,0,1 };
    float angle = PI / 2;
    rot[0] = cos(angle);
    rot[2] = sin(angle);
    rot[6] = -sin(angle);
    rot[8] = cos(angle);

    //求映射Map
    cv::Mat mapImg = cv::Mat::zeros(outHeight, outWidth, CV_32FC2);
    rectifyMap(mapImg, inWidth, inHeight,rot, outWidth, outHeight, FOV, radius);
    //remap得到经纬度图像
    cv::Mat dstImg = cv::Mat::zeros(outHeight, outWidth, CV_8UC3);
    remap(input_image, dstImg, mapImg, inHeight, inWidth, outHeight, outWidth);
    return dstImg;
}


void FisheyeCameraUndistortion::rectifyMap(cv::Mat &mapImg, const int inWidth, const int inHeight,
                                           const float* rot, const int outWidth, const int outHeight,
                                           const float FOV, const float radius)
{
    float cx = inWidth/2.0;
    float cy = inHeight/2.0;

    float* pMapData = (float*)mapImg.data;
    for (int j = 0; j < outHeight; j++)
    {
        float theta1 = j*PI / outHeight;
        float sinTheta1 = sin(theta1);
        float z1 = cos(theta1);

        for (int i = 0; i < outWidth; i++)
        {
            float fi1 = 2 * PI - i* 2*PI / outWidth;
            float x1 = sinTheta1*cos(fi1);
            float y1 = sinTheta1*sin(fi1);

            //归一化三维坐标
            float x2 = rot[0] * x1 + rot[1] * y1 + rot[2] * z1;
            float y2 = rot[3] * x1 + rot[4] * y1 + rot[5] * z1;
            float z2 = rot[6] * x1 + rot[7] * y1 + rot[8] * z1;
            float norm = sqrt(x2*x2 + y2*y2 + z2*z2);
            x2 /= norm;
            y2 /= norm;
            z2 /= norm;

            //球面坐标系转换
            float theta2 = acos(z2)*180/PI;
            float fi2 = atan2(y2, x2);

            if (theta2 <= (FOV / 2) && theta2 >= 0)
            {
                //球面到鱼眼
                float radius2 =radius* theta2 / (FOV / 2);
                float u = (radius2*cos(fi2) + cx);
                float v = (radius2*sin(fi2) + cy);
                if (u >= 0 && u < inWidth - 1 && v >= 0 && v < inHeight - 1)
                {
                    pMapData[j*outWidth * 2 + 2 * i + 0] = u;
                    pMapData[j*outWidth * 2 + 2 * i + 1] = v;
                }
                else
                {
                    pMapData[j*outWidth * 2 + 2 * i + 0] = 0;
                    pMapData[j*outWidth * 2 + 2 * i + 1] = 0;
                }
            }
            else
            {
                pMapData[j*outWidth * 2 + 2 * i + 0] = 0;
                pMapData[j*outWidth * 2 + 2 * i + 1] = 0;
            }
        }
    }
}

void FisheyeCameraUndistortion::remap(const cv::Mat& srcImg, cv::Mat& dstImg, const cv::Mat& mapImg,
                                      int inHeight, int inWidth, int outHeight, int outWidth)
{
    uchar* pSrcData = (uchar*)srcImg.data;
    uchar* pDstData = (uchar*)dstImg.data;
    float* pMapData = (float*)mapImg.data;

    for (int j = 0; j < outHeight; j++)
    {
        for (int i = 0; i < outWidth; i++)
        {
            int idx = j*outWidth * 2 + i * 2;
            float u = pMapData[idx + 0];
            float v = pMapData[idx + 1];

            int u0 = floor(u);
            int v0 = floor(v);
            float dx = u - u0;
            float dy = v - v0;
            float weight1 = (1 - dx)*(1 - dy);
            float weight2 = dx*(1 - dy);
            float weight3 = (1 - dx)*dy;
            float weight4 = dx*dy;

            if (u0 >= 0 && v0 >= 0 && (u0 + 1) < inWidth && (v0 + 1) < inHeight)
            {
                float B = weight1*pSrcData[v0*inWidth * 3 + u0 * 3 + 0] + weight2*pSrcData[v0*inWidth * 3 + (u0 + 1) * 3 + 0] +
                    weight3*pSrcData[(v0 + 1)*inWidth * 3 + u0 * 3 + 0] + weight4*pSrcData[(v0 + 1)*inWidth * 3 + (u0 + 1) * 3 + 0];

                float G = weight1*pSrcData[v0*inWidth * 3 + u0 * 3 + 1] + weight2*pSrcData[v0*inWidth * 3 + (u0 + 1) * 3 + 1] +
                    weight3*pSrcData[(v0 + 1)*inWidth * 3 + u0 * 3 + 1] + weight4*pSrcData[(v0 + 1)*inWidth * 3 + (u0 + 1) * 3 + 1];

                float R = weight1*pSrcData[v0*inWidth * 3 + u0 * 3 + 2] + weight2*pSrcData[v0*inWidth * 3 + (u0 + 1) * 3 + 2] +
                    weight3*pSrcData[(v0 + 1)*inWidth * 3 + u0 * 3 + 2] + weight4*pSrcData[(v0 + 1)*inWidth * 3 + (u0 + 1) * 3 + 2];

                int idxResult = j*outWidth * 3 + i * 3;
                pDstData[idxResult + 0] = uchar(B);
                pDstData[idxResult + 1] = uchar(G);
                pDstData[idxResult + 2] = uchar(R);
            }
        }
    }
}
