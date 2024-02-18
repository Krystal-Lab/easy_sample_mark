#ifndef LIDARRADARMANUALWINDOW_H
#define LIDARRADARMANUALWINDOW_H

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QScrollArea>
#include <QGroupBox>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QHeaderView>
#include <QTableWidget>
#include <QImage>
#include <QLineEdit>
#include <QSlider>
#include "baseAlgorithm/common_transform.h"
#include "baseAlgorithm/coordinate_transform.h"
#include "utilityGUI/customWindow/mytextbrowser.h"
#include "saveData/radardataloader.h"
#include "pointcloudviewer.h"

#include <opencv2/core.hpp>

class LidarRadarManualWindow : public QDialog
{
    Q_OBJECT
public:
    explicit LidarRadarManualWindow(QDialog *parent = nullptr);
    ~LidarRadarManualWindow();

signals:

public slots:
    void slotLoadPointCloud();
    void slotLoadRadar();
    void slotLoadExtrinsic();

    void slotShowCalibration();
    void slotStepChange(double value);
    void slotDegreeParamChange(double value);
    void slotTransParamChange(double value);
    void slotResetCalibration();
    void slotSaveResult();

private:

    QScrollArea *cloudScrollArea;
    PointCloudViewer *drawPointCloud;
    MyTextBrowser *commandText;//输出黑匣子指令

    QLineEdit *pointCloudText;
    QPushButton *openPointCloudButton;
    QLineEdit *radarText;
    QPushButton *openRadarButton;
    QLineEdit *extrinsicText;
    QPushButton *openExtrinsicButton;

    QPushButton *resetButton;
    QPushButton *saveResultButton;

    QGroupBox *paramGroundBox;
    QLabel *scaleDegreeLabel;
    QDoubleSpinBox *scaleDegreeBox;
    QLabel *scaleTransLabel;
    QDoubleSpinBox *scaleTransBox;
    QLabel *xDegreeLabel;
    QDoubleSpinBox *xDegreeBox;
    QLabel *yDegreeLabel;
    QDoubleSpinBox *yDegreeBox;
    QLabel *zDegreeLabel;
    QDoubleSpinBox *zDegreeBox;
    QLabel *xTransLabel;
    QDoubleSpinBox *xTransBox;
    QLabel *yTransLabel;
    QDoubleSpinBox *yTransBox;
    QLabel *zTransLabel;
    QDoubleSpinBox *zTransBox;

    QString openDataDir;
    bool isInit;

    Eigen::Matrix4f calibration_matrix_;
    Eigen::Matrix4f orign_calibration_matrix_;
    Eigen::Matrix4f modification_matrix_;
    std::vector<cv::Point3f> point3DList;

    Transform transform_process;

    RadarDataLoader radar_dataloader;

    void init();
    void initUI();
    void initConnect();

    void calibrationInit();

    bool loadExtrinsic(const QString &filePath);
};

#endif // LIDARRADARMANUALWINDOW_H
