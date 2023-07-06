#ifndef RADARCAMERAMANUALWINDOW_H
#define RADARCAMERAMANUALWINDOW_H

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
#include "helpers/convertcvqimage.h"
#include "baseAlgorithm/common_transform.h"
#include "baseAlgorithm/coordinate_transform.h"
#include "utilityGUI/customWindow/mytextbrowser.h"
#include "saveMarkData/calibrationparamload.h"
#include "imagelabel.h"

#include <calibration/radar_camera/projector_radar.hpp>

#include <opencv2/core.hpp>

class RadarCameraManualWindow : public QDialog
{
    Q_OBJECT
public:
    explicit RadarCameraManualWindow(QDialog *parent = nullptr);
    ~RadarCameraManualWindow();

signals:

public slots:
    void slotLoadIntrinsic();
    void slotLoadHomography();
    void slotLoadExtrinsic();
    void slotLoadImage();
    void slotLoadRadar();
    void slotSelectPoint();
    void slotShowCalibration();
    void slotStepChange(double value);
    void slotDegreeParamChange(double value);
    void slotTransParamChange(double value);
    void slotScaleImageShow(int value);
    void slotResetCalibration();
    void slotSaveResult();

private:

    ImageLabel *imageShow;
    QScrollArea *imageScrollArea;//滚动区域
    ImageLabel *birdImageShow;
    QScrollArea *birdScrollArea;//滚动区域
    MyTextBrowser *commandText;//输出黑匣子指令

    QLineEdit *intrinsicText;
    QPushButton *openIntrinsicButton;
    QLineEdit *homographyText;
    QPushButton *openHomographyButton;
    QLineEdit *extrinsicText;
    QPushButton *openExtrinsicButton;
    QPushButton *openImageButton;
    QPushButton *openRadarButton;
    QPushButton *selectPointButton;
    QPushButton *showCalibrationButton;

    QLabel *scaleShowLabel;
    QSlider *scaleShowBox;
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

    QImage currentImage;
    QImage birdViewImage;

    cv::Mat currentMat;

    ConvertCVQImage convertImage;

    Projector projector;

    QString openDataDir;
    bool isLoadIntrinsic;
    bool isLoadHomography;
    bool isLoadExtrinsic;
    bool isInit;

    cv::Mat cameraInstrinsics;
    cv::Mat distortionCoefficients;
    cv::Mat homography;
    Eigen::Matrix4f calibration_matrix_;
    Eigen::Matrix4f orign_calibration_matrix_;
    Eigen::Matrix4f modification_matrix_;
    std::vector<cv::Point3f> point3DList;

    QList<QPoint> selectPointList;

    CalibrationParamLoad paramLoad;

    void init();
    void initUI();
    void initConnect();

    void updateButtonStatus();
    void calibrationInit();

    bool loadExtrinsic(const QString &filePath);
    bool loadRadarData(const QString &filePath);
};

#endif // RADARCAMERAMANUALWINDOW_H
