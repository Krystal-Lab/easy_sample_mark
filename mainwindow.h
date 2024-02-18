#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAction>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QAction>
#include <QComboBox>
#include <QStackedWidget>
#include "videoTools/segmentationlabelconvertwindow.h"
#include "videoTools/fromvideotopicturewindow.h"
#include "videoTools/frompicturetovideowindow.h"
#include "videoTools/videocuttingwindow.h"
#include "videoTools/videocroppingwindow.h"
#include "videoTools/imageconverterwindow.h"
#include "videoTools/qcamerawindow.h"
#include "pcTools/pcdconverterwindow.h"
#include "pcTools/pcdfilterwindow.h"
#include "calibrationTools/birdviewprocess.h"
#include "calibrationTools/cameraintrinsicswindow.h"
#include "calibrationTools/cameraverificationwindow.h"
#include "calibrationTools/radarcameramanualwindow.h"
#include "calibrationTools/lidarradarmanualwindow.h"
#include "autoSampleMark/autodetection2dwindow.h"
#include "drawShape/myshape.h"
#include "controlwindow.h"
#include "imagecontrolwindow.h"
#include "videocontrolwindow.h"
#include "imagesegmentcontrolwindow.h"
#include "ocrcontrolwindow.h"
#include "imagetrackingcontrolwindow.h"
#include "pclcontrolwindow.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

signals:
    void signalManualMarkParamterChanged();

public slots:
    //file
    void slotOpenImageDir();
    void slotOpenVideoDir();
    void slotOpenImageSegmentDir();
    void slotOpenOCRImageDir();
    void slotOpenImageTrackingDir();
    void slotOpenPCDDir();
    //setting
    void slotManualMarkParamterConfig();
    void slotSegmentMarkParamterConfig();
    void slotAutoMarkParamterConfig();
    void slotVideoMarkParamterConfig();

    void slotPointCloudParamterConfig();
    //autoMark
    void slotAutoSampleMark();
    //tool
    void slotSegLabelConvert();

    void slotVideoToPicture();
    void slotVideoFromPicture();
    void slotVideoCropping();
    void slotVideoCutting();
    void slotImageConverter();
    void slotCamera();

    void slotPcdConverter();
    void slotPcdFilter();

    // calibration
    void slotBirdViewProcess();
    void slotCameraIntrinsics();
    void slotCameraVerification();
    void slotRadarCameraManual();
    void slotLidarRadarManual();

    //about
    void slotAbout();
    void slotUserManual();

    //shapeTool
    void slotSelectMarkShape(const QString &text);

    void slotIsMarkStatus(bool isMark);

    void slotCloseOtherWindow(QString flag);

protected:
    void closeEvent(QCloseEvent *event);

private:

    //Action
    //file
    QAction *openImageDirAction;
    QAction *openSegmentImageDirAction;
    QAction *openOCRImageDirAction;
    QAction *openImageTrackingDirAction;
    QAction *openVideoDirAction;
    QAction *openPCDDirAction;
    QAction *exitAction;
    //setting
    QAction *manualParamterAction;
    QAction *segmentParamterAction;
    QAction *autoParamterAction;
    QAction *videoMarkParamterAction;
    QAction *pointcloudParamterAction;
    //autoMark
    QAction *autoDet2dAction;
    //tool
    QAction *segLabelConvertAction;
    QAction *videoToPictureAction;
    QAction *videoFromPictureAction;
    QAction *videoCuttingAction;
    QAction *videoCroppingAction;
    QAction *imageConverterAction;
    QAction *cameraAction;
    QAction *pcdConverterAction;
    QAction *pcdFilterAction;
    //calibration
    QAction *birdViewCalibratAction;
    QAction *cameraIntrinsicsAction;
    QAction *cameraVerificationAction;
    QAction *radarCameraManualCalibratAction;
    QAction *lidarRadarManualCalibratAction;
    //about
    QAction *aboutAction;
    QAction *userManualAction;

    //Menu
    QMenu *fileMenu;
    QMenu *settingMenu;
    QMenu *autoMarkMenu;
    QMenu *toolMenu;
    QMenu *calibrationMenu;
    QMenu *aboutMenu;

    //ToolBar
    QToolBar *fileTool;
    QToolBar *autoMarkTool;
    QToolBar *shapeTool;

    //shapeTool
    QWidget *shapeWidget;
    QLabel *shapeLabel;
    QComboBox *shapeBox;

    QStackedWidget *centerWidget;
    QList<ControlWindow *> markWindow;

private:
    AutoDetection2DWindow *autoDetection2DWindow;

    SegmentationLabelConvertWindow *segLabelConvertWindow;
    FromVideoToPictureWindow *videoToPictureWindow;
    FromPictureToVideoWindow *videoFromPictureWindow;
    VideoCuttingWindow *videoCuttingWindow;
    VideoCroppingWindow *videoCroppingWindow;
    ImageConverterWindow *imageConverterWindow;
    QCameraWindow *cameraWindow;

    PCDConverterWindow *pcdConverterWindow;
    PCDFilterWindow *pcdFilterWindow;

    // calibration
    BirdViewProcess *birdViewPorcess;
    CameraIntrinsicsWindow * cameraIntrinsicsWindow;
    CameraVerificationWindow *cameraVerificationWindow;
    RadarCameraManualWindow *radarCameraManualWindow;
    LidarRadarManualWindow *lidarRadarManualWindow;

    MarkDataType loadDataType;
    MyShape imgShape;

    QString openDataDir;

private:
    void initData();
    void initAction();
    void initMenuBar();
    void initToolBar();
    void initUI();
    void initConnect();

    void initImageMarkShape();
    void initSegmentMarkShape();
    void initOCRMarkShape();
    void initImageTrackingMarkShape();
    void initPointCloudMarkShape();

};

#endif // MAINWINDOW_H
