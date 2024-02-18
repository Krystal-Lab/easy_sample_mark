#ifndef POINTCLOUDVIEWER_H
#define POINTCLOUDVIEWER_H

// Qt
#include <QEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QCursor>
#include <QString>

#include "saveData/pointcloudreader.h"

#include <cfloat>
#include <pcl/visualization/eigen.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
#include <pcl/visualization/pcl_plotter.h>
#endif
#include <pcl/visualization/point_picking_event.h>
#include <pcl/search/kdtree.h>
#include <vtkPolyDataReader.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#ifdef WIN32
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);
#endif
#include <QVTKWidget.h>

#include "baseAlgorithm/common_transform.h"

class PointCloudViewer : public QVTKWidget
{
    Q_OBJECT

public:

    typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
    typedef ColorHandler::Ptr ColorHandlerPtr;
    typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

    typedef pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2> GeometryHandler;
    typedef GeometryHandler::Ptr GeometryHandlerPtr;
    typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

    PointCloudViewer(QWidget *parent = 0);
    ~PointCloudViewer();

    void setIsSelect(bool isSelect);
    int setNewPointCloud(const QString &pcdFilePath);
    void clearPoints();
    void getPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &result);
    void setRadarPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &points);

protected:
    void enterEvent(QEvent *e);
    void leaveEvent(QEvent *e);
    void mouseDoubleClickEvent(QMouseEvent *e);
    void clickedPointCallback(const pcl::visualization::PointPickingEvent& event);
    void clickedAreaCallback(const pcl::visualization::AreaPickingEvent& event);

private:
    void drawTypeColorPointCloud();

    void drawCircle();

    void convertRGBPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    void showCoordinateSystem(const Eigen::Affine3f& tf, int index, double scale=1.0);

    void initData();
    void initConnect();
    void initCloud();

private:

    PointCloudReader pcReader;
    ColorHandlerPtr colorHandler;
    GeometryHandlerPtr geometryHandler;
    pcl::visualization::PCLVisualizer::Ptr viewer_;

    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCLPointCloud2::Ptr srcCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clickedPoints;

    QString sampleClass;

    QCursor myDrawCursor;
    bool isSelect;

    std::set<int> frame_names_;

    Transform transform;
};

#endif // POINTCLOUDVIEWER_H
