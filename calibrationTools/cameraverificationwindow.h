#ifndef CAMERAVERIFICATIONWINDOW_H
#define CAMERAVERIFICATIONWINDOW_H

#include <QDialog>
#include <QLabel>
#include <QScrollArea>
#include <QImage>
#include <QGroupBox>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QListWidget>
#include <QCheckBox>
#include <QLineEdit>
#include "helpers/convertcvqimage.h"
#include "utilityGUI/customWindow/mytextbrowser.h"
#include "imagelabel.h"
#include "saveData/calibrationparamload.h"
#include "calibration/camera_intrinsic/cameraintrinscalibration.h"
#include "calibration/camera_intrinsic/fisheyecameraundistortion.h"
#include "calibration/camera_intrinsic/CalibrationHarp.hpp"

class CameraVerificationWindow : public QDialog
{
    Q_OBJECT
public:
    explicit CameraVerificationWindow(QDialog *parent = nullptr);
    ~CameraVerificationWindow();

signals:

public slots:
    void slotOpenImageDir();
    void slotImageItem(QListWidgetItem *item);
    void slotVerification();
    void slotSaveImage();

    void slotLoadCameraIntrinsic();

private:
    void init();
    void initUI();
    void initConnect();

private:

    QLineEdit *intrinsicText;
    QPushButton *openIntrinsicButton;
    QLabel *cameraModelLabel;
    QComboBox *cameraModelBox;
    QLabel *undistortModelLabel;
    QComboBox *undistortModelBox;

    ImageLabel *imageShow;
    QScrollArea *scrollArea;//滚动区域
    MyTextBrowser *commandText;//输出黑匣子指令

    QListWidget *imageListWidget;
    QPushButton *openDirButton;
    QPushButton *verificationButton;
    QPushButton *saveButton;

    QImage currentImage;
    QString currentImagePath;

    cv::Mat rgbFrame;

    QList<QString> processDataList;
    QString openDataDir;

    bool isLoadIntrinsic;

    cv::Mat cameraInstrinsics;
    cv::Mat distortionCoefficients;
    cv::Point2f scale_focal;
    cv::Point2f shift_center;

    CalibrationParamLoad paramLoad;
    CameraIntrinsCalibration calibrationProcess;
    FisheyeCameraUndistortion fisheyeCameraProcess;

    CalibrationHarp distortionMeasurement;
};

#endif // CAMERAVERIFICATIONWINDOW_H
