#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "pointcloudviewer.h"
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <boost/bind.hpp>
#include"sampleMarkParam/pointcloudparamterconfig.h"

PointCloudViewer::PointCloudViewer(QWidget *parent) : QVTKWidget(parent)
{
    initData();
    initConnect();
}

PointCloudViewer::~PointCloudViewer()
{

}

void PointCloudViewer::setIsSelect(bool isSelect)
{
    this->isSelect = isSelect;
}

int PointCloudViewer::setNewPointCloud(const QString &pcdFilePath)
{
    int errorCode = 0;
    errorCode = pcReader.pcdRead(pcdFilePath.toStdString(), srcCloud,
                                     origin, orientation);
    drawTypeColorPointCloud();
    return errorCode;
}

void PointCloudViewer::clearPoints()
{
    viewer_->removeAllShapes();
    viewer_->removePointCloud("srcCloud");
    if(clickedPoints->size() > 0)
    {
        clickedPoints->clear();
        viewer_->updatePointCloud(clickedPoints, "clickedPoints");
        this->update();
    }
}

void PointCloudViewer::getPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &result)
{
    result->clear();
    for(const pcl::PointXYZRGB &point: clickedPoints->points)
    {
        pcl::PointXYZI tempPoint;
        tempPoint.x = point.x;
        tempPoint.y = point.y;
        tempPoint.z = point.z;
        tempPoint.intensity = 0;
        result->push_back(tempPoint);
    }
}

void PointCloudViewer::setRadarPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &points)
{
    int index = 0;
    viewer_->removeAllShapes();
    for(const pcl::PointXYZ &point: points->points)
    {
        QString boxId = QString("box_%1").arg(index);
        Eigen::AngleAxisf rotation_vector(0, Eigen::Vector3f(0, 0, 1));
        Eigen::Vector3f center(point.x, point.y, point.z);
        this->viewer_->addCube(center, Eigen::Quaternionf(rotation_vector), 0.3, 0.3, 0.3, boxId.toStdString());
        this->viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, \
                                            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, boxId.toStdString());
        this->viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, boxId.toStdString());
        this->viewer_->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, boxId.toStdString());
        index++;
    }
    this->update();
}

void PointCloudViewer::enterEvent(QEvent *e)
{
    if(isSelect)
    {
        setCursor(myDrawCursor);
    }
    QVTKWidget::enterEvent(e);
}

void PointCloudViewer::leaveEvent(QEvent *e)
{
    QVTKWidget::leaveEvent(e);
}

void PointCloudViewer::mouseDoubleClickEvent(QMouseEvent *e)
{
    QVTKWidget::mouseDoubleClickEvent(e);
}

void PointCloudViewer::clickedPointCallback(const pcl::visualization::PointPickingEvent& event)
{
    if(isSelect)
    {
        bool isIn = false;
        pcl::PointXYZRGB currentPoint;
        if (event.getPointIndex() == -1)
            return;
        event.getPoint(currentPoint.x, currentPoint.y, currentPoint.z);
        currentPoint.r = 0;
        currentPoint.g = 0;
        currentPoint.b = 0;
        for(size_t loop = 0; loop < this->clickedPoints->size(); loop++)
        {
            if(this->clickedPoints->points[loop].x != currentPoint.x
                    && this->clickedPoints->points[loop].y != currentPoint.y
                    && this->clickedPoints->points[loop].z != currentPoint.z)
            {
                isIn = true;
                break;
            }
        }
        if(isIn || this->clickedPoints->size() <= 0)
            this->clickedPoints->push_back(currentPoint);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(clickedPoints, 255, 0, 0);

        this->viewer_->removePointCloud("clickedPoints");
        this->viewer_->addPointCloud(this->clickedPoints, red, "clickedPoints");
        this->viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clickedPoints");

        this->update();

        std::cout << "point:" << currentPoint.x << " " << currentPoint.y << " " << currentPoint.z << std::endl;
    }
}

void PointCloudViewer::clickedAreaCallback(const pcl::visualization::AreaPickingEvent& event)
{
  if(isSelect)
  {
      std::vector< int > indices;
      if(event.getPointsIndices(indices)==-1)
        return;

      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromPCLPointCloud2(*srcCloud, cloud);

      for (int i = 0; i < indices.size(); ++i)
      {
        bool isIn = false;
        pcl::PointXYZRGB currentPoint;
        currentPoint.x = cloud.points.at(indices[i]).x;
        currentPoint.y = cloud.points.at(indices[i]).y;
        currentPoint.z = cloud.points.at(indices[i]).z;
        currentPoint.r = 0;
        currentPoint.g = 0;
        currentPoint.b = 0;

        for(size_t loop = 0; loop < this->clickedPoints->size(); loop++)
        {
            if(this->clickedPoints->points[loop].x != currentPoint.x
                    && this->clickedPoints->points[loop].y != currentPoint.y
                    && this->clickedPoints->points[loop].z != currentPoint.z)
            {
                isIn = true;
                break;
            }
        }
        if(isIn || this->clickedPoints->size() <= 0)
            this->clickedPoints->push_back(currentPoint);
      }

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(clickedPoints, 255, 0, 0);

      viewer_->removePointCloud("clickedPoints");
      viewer_->addPointCloud(clickedPoints, red, "clickedPoints");
      viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "clickedPoints");

      this->update();

      for (int i = 0; i < clickedPoints->points.size(); i++)
        std::cout << clickedPoints->points[i].x << ", " << clickedPoints->points[i].y << ", " << clickedPoints->points[i].z << ", " << std::endl;
      std::cout << "new size = " << clickedPoints->points.size() << std::endl;
  }
}

void PointCloudViewer::drawTypeColorPointCloud()
{
    viewer_->removePointCloud("srcCloud");
    geometryHandler.reset(new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>(srcCloud));

    if(PointCloudParamterConfig::getColorRenderType() == "intensity")
    {
        colorHandler.reset(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>(srcCloud,
                                                                                                           "intensity"));
    }
    else if(PointCloudParamterConfig::getColorRenderType() == "x" ||
            PointCloudParamterConfig::getColorRenderType() == "y" ||
            PointCloudParamterConfig::getColorRenderType() == "z")
    {
        colorHandler.reset(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>(srcCloud,
                                                                                                           PointCloudParamterConfig::getColorRenderType().toStdString()));
    }
    else if(PointCloudParamterConfig::getColorRenderType() == "rgb" && \
             PointCloudParamterConfig::getFieldsNumber() == 6)
    {
        colorHandler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(srcCloud));
    }
    else
    {
        colorHandler.reset(new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(srcCloud));
    }

    viewer_->addPointCloud(srcCloud, geometryHandler, colorHandler, \
                          origin, orientation, "srcCloud", 0);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, \
                                             PointCloudParamterConfig::getPointSize(), "srcCloud", 0);

//    viewer_->removeAllShapes();
//    viewer_->addCube(0, 1.0, 0, 1.0, 0, 1.0, 1, 0, 0, "cube1", 0);
//    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, \
//                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube1");
//    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 255, 0, "cube1");
//    viewer_->addCube(0, 3.0, 0, 3.0, 0, 3.0, 1, 0, 0, "cube2", 0);
//    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, \
//                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube2");
//    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 255, 0, "cube2");

    this->update();
}

void PointCloudViewer::drawCircle()
{
    pcl::ModelCoefficients circleCoeff1;
    circleCoeff1.values.resize(3);
    circleCoeff1.values[0] = 0;
    circleCoeff1.values[1] = 0;
    circleCoeff1.values[2] = 50.0f;
    viewer_->addCircle(circleCoeff1, "circle1", 0);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, \
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "circle1");
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "circle1");

    pcl::ModelCoefficients circleCoeff2;
    circleCoeff2.values.resize(3);
    circleCoeff2.values[0] = 0;
    circleCoeff2.values[1] = 0;
    circleCoeff2.values[2] = 100.0f;
    viewer_->addCircle(circleCoeff2, "circle2", 0);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, \
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "circle2");
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 255, 0, "circle2");
}

void PointCloudViewer::convertRGBPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    float rScale = 1;
    float gScale = 0.5;
    float bScale = 0.1;
    rgbCloud->clear();
    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      pcl::PointXYZRGB point;
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;
      point.r = static_cast<unsigned char>(cloud->points[i].intensity * rScale);
      point.g = static_cast<unsigned char>(cloud->points[i].intensity * gScale);
      point.b = static_cast<unsigned char>(cloud->points[i].intensity * bScale);
      rgbCloud->push_back(point);
    }
}

/**
 * @brief show coordinate in the viewer
 * @param [in] tf    the coordinate transformation
 * @param [in] index the coordinate id
 * @param [in] scale the scale, default 1.0
 */
void PointCloudViewer::showCoordinateSystem(const Eigen::Affine3f& tf, int index, double scale)
{
    std::string frame_name = std::string("frame_") + std::to_string(index);
    if(frame_names_.count(index))
    {
        viewer_->updateCoordinateSystemPose(frame_name, tf);
    }
    else
    {
        frame_names_.insert(index);
        viewer_->addCoordinateSystem(scale, tf, frame_name);
    }
}

void PointCloudViewer::initData()
{
    viewer_.reset(new pcl::visualization::PCLVisualizer("cloud", false));
    initCloud();

    colorHandler.reset(new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(srcCloud));
    geometryHandler.reset(new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>(srcCloud));
    //geometryHandler.reset(new pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<pcl::PCLPointCloud2>(srcCloud));
    viewer_->addPointCloud(srcCloud, geometryHandler, colorHandler, origin, orientation, "srcCloud", 0);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, \
                                             PointCloudParamterConfig::getPointSize(), "srcCloud", 0);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, "srcCloud");

    this->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(this->GetInteractor (), this->GetRenderWindow ());

    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();
    viewer_->resetCamera();

    viewer_->registerPointPickingCallback(boost::bind(&PointCloudViewer::clickedPointCallback, this, _1));
    viewer_->registerAreaPickingCallback(boost::bind(&PointCloudViewer::clickedAreaCallback, this, _1));

    // Set whether or not we should be using the vtkVertexBufferObjectMapper
    viewer_->setUseVbos(true);

    this->update();

    sampleClass = "All";

    isSelect = false;
    myDrawCursor = QCursor(QPixmap(tr(":/images/images/cross.png")));
}

void PointCloudViewer::initConnect()
{

}

void PointCloudViewer::initCloud()
{
    srcCloud.reset(new pcl::PCLPointCloud2);
    rgbCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    clickedPoints.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}
