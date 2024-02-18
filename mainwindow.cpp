#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "mainwindow.h"
#include <QApplication>
#include <QDesktopServices>
#include <QMessageBox>
#include <QDebug>
#include "helpers/dirprocess.h"
#include "paramWindow/manualparamterconfigwindow.h"
#include "paramWindow/videomarkparamterwindow.h"
#include "paramWindow/segmentparamterconfigwindow.h"
#include "paramWindow/pointcloudmarkparamterwindow.h"
#include "paramWindow/autoparamterconfigwindow.h"
#include "sampleMarkParam/pointcloudparamterconfig.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    initData();
    initMenuBar();
    initToolBar();
    initUI();
    initConnect();
}

MainWindow::~MainWindow()
{
    markWindow[loadDataType]->saveClassConfig();
    centerWidget->deleteLater();
}

void MainWindow::slotOpenImageDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initImageMarkShape();
        processDataList = dirProcess.getDirFileName(this->openDataDir);
        markWindow[loadDataType]->saveMarkDataList();
        markWindow[loadDataType]->saveClassConfig();
        loadDataType = MarkDataType::IMAGE;
        markWindow[loadDataType]->setDrawShape(this->shapeBox->currentData().toInt());
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotOpenVideoDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initImageTrackingMarkShape();
        processDataList = dirProcess.getDirFileName(this->openDataDir);
        markWindow[loadDataType]->saveMarkDataList();
        markWindow[loadDataType]->saveClassConfig();
        loadDataType = MarkDataType::VIDEO;
        markWindow[loadDataType]->setDrawShape(this->shapeBox->currentData().toInt());
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotOpenImageSegmentDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initSegmentMarkShape();
        processDataList = dirProcess.getDirFileName(this->openDataDir);
        markWindow[loadDataType]->saveMarkDataList();
        markWindow[loadDataType]->saveClassConfig();
        loadDataType = MarkDataType::SEGMENT;
        markWindow[loadDataType]->setDrawShape(this->shapeBox->currentData().toInt());
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotOpenOCRImageDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initOCRMarkShape();
        processDataList = dirProcess.getDirFileName(this->openDataDir);
        markWindow[loadDataType]->saveMarkDataList();
        markWindow[loadDataType]->saveClassConfig();
        loadDataType = MarkDataType::OCR;
        markWindow[loadDataType]->setDrawShape(this->shapeBox->currentData().toInt());
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotOpenImageTrackingDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initImageTrackingMarkShape();
        processDataList = dirProcess.getDirFileName(this->openDataDir);
        markWindow[loadDataType]->saveMarkDataList();
        markWindow[loadDataType]->saveClassConfig();
        loadDataType = MarkDataType::IMAGE_TRACKING;
        markWindow[loadDataType]->setDrawShape(this->shapeBox->currentData().toInt());
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotOpenPCDDir()
{
    DirProcess dirProcess;
    QList<QString> processDataList;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        initPointCloudMarkShape();
        if(PointCloudParamterConfig::getFileType() == PointCloudFileType::PCD_FILE)
        {
            dirProcess.getDirAllFileName(this->openDataDir, "*.pcd", processDataList);
        }
        else if(PointCloudParamterConfig::getFileType() == PointCloudFileType::BIN_FILE)
        {
            dirProcess.getDirAllFileName(this->openDataDir, "*.bin", processDataList);
        }
        else if(PointCloudParamterConfig::getFileType() == PointCloudFileType::PLY_FILE)
        {
            dirProcess.getDirAllFileName(this->openDataDir, "*.ply", processDataList);
        }
        else if(PointCloudParamterConfig::getFileType() == PointCloudFileType::OBJ_FILE)
        {
            dirProcess.getDirAllFileName(this->openDataDir, "*.obj", processDataList);
        }
        markWindow[loadDataType]->saveMarkDataList();
        markWindow[loadDataType]->saveClassConfig();
        loadDataType = MarkDataType::PCD;
        markWindow[loadDataType]->setMarkDataList(this->openDataDir, processDataList, loadDataType);
        centerWidget->setCurrentIndex(loadDataType);
    }
}

void MainWindow::slotManualMarkParamterConfig()
{
    int result = 0;
    ManualParamterConfigWindow *manualMarkParamterWindow = new ManualParamterConfigWindow();
    manualMarkParamterWindow->setModal(true);
    result = manualMarkParamterWindow->exec();
    if(result == QDialog::Accepted)
    {
        emit signalManualMarkParamterChanged();
    }
    manualMarkParamterWindow->deleteLater();;
    manualMarkParamterWindow = nullptr;
}

void MainWindow::slotAutoMarkParamterConfig()
{
    int result = 0;
    AutoParamterConfigWindow *window = new AutoParamterConfigWindow();
    window->setModal(true);
    result = window->exec();
    if(result == QDialog::Accepted)
    {
        ;
    }
    window->deleteLater();
    window = nullptr;
}

void MainWindow::slotVideoMarkParamterConfig()
{
    int result = 0;
    VideoMarkParamterWindow *window = new VideoMarkParamterWindow();
    window->setModal(true);
    result = window->exec();
    if(result == QDialog::Accepted)
    {
        ;
    }
    window->deleteLater();
    window = nullptr;
}

void MainWindow::slotSegmentMarkParamterConfig()
{
    int result = 0;
    SegmentParamterConfigWindow *window = new SegmentParamterConfigWindow();
    window->setModal(true);
    result = window->exec();
    if(result == QDialog::Accepted)
    {
        ;
    }
    window->deleteLater();
    window = nullptr;
}

void MainWindow::slotPointCloudParamterConfig()
{
    int result = 0;
    PointCloudMarkParamterWindow *window = new PointCloudMarkParamterWindow();
    window->setModal(true);
    result = window->exec();
    if(result == QDialog::Accepted)
    {
        ;
    }
    window->deleteLater();
    window = nullptr;
}

void MainWindow::slotAutoSampleMark()
{
    if(autoDetection2DWindow == nullptr)
    {
        autoDetection2DWindow = new AutoDetection2DWindow();
        connect(autoDetection2DWindow, &AutoDetection2DWindow::signalCloseAutoSampleMarkWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    autoDetection2DWindow->show();
}

void MainWindow::slotSegLabelConvert()
{
    if(segLabelConvertWindow == nullptr)
    {
        segLabelConvertWindow = new SegmentationLabelConvertWindow();
        connect(segLabelConvertWindow, &SegmentationLabelConvertWindow::signalCloseSegLabelConverterWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    segLabelConvertWindow->show();
}

void MainWindow::slotVideoToPicture()
{
    if(videoToPictureWindow == nullptr)
    {
        videoToPictureWindow = new FromVideoToPictureWindow();
        connect(videoToPictureWindow, &FromVideoToPictureWindow::signalCloseVideoToPictureWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    videoToPictureWindow->show();
}

void MainWindow::slotVideoFromPicture()
{
    if(videoFromPictureWindow == nullptr)
    {
        videoFromPictureWindow = new FromPictureToVideoWindow();
        connect(videoFromPictureWindow, &FromPictureToVideoWindow::signalCloseVideoFromPictureWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    videoFromPictureWindow->show();
}

void MainWindow::slotVideoCropping()
{
    if(videoCroppingWindow == nullptr)
    {
        videoCroppingWindow = new VideoCroppingWindow();
        connect(videoCroppingWindow, &VideoCroppingWindow::signalCloseVideoCroppingWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    videoCroppingWindow->show();
}

void MainWindow::slotVideoCutting()
{
    if(videoCuttingWindow == nullptr)
    {
        videoCuttingWindow = new VideoCuttingWindow();
        connect(videoCuttingWindow, &VideoCuttingWindow::signalCloseVideoCuttingWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    videoCuttingWindow->show();
}

void MainWindow::slotImageConverter()
{
    if(imageConverterWindow == nullptr)
    {
        imageConverterWindow = new ImageConverterWindow();
        connect(imageConverterWindow, &ImageConverterWindow::signalCloseImageConverterWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    imageConverterWindow->show();
}

void MainWindow::slotCamera()
{
    if(cameraWindow == nullptr)
    {
        cameraWindow = new QCameraWindow();
        connect(cameraWindow, &QCameraWindow::signalCloseCameraWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    cameraWindow->show();
}

void MainWindow::slotPcdConverter()
{
    if(pcdConverterWindow == nullptr)
    {
        pcdConverterWindow = new PCDConverterWindow();
        connect(pcdConverterWindow, &PCDConverterWindow::signalClosePCDConverterWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    pcdConverterWindow->show();
}

void MainWindow::slotPcdFilter()
{
    if(pcdFilterWindow == nullptr)
    {
        pcdFilterWindow = new PCDFilterWindow();
        connect(pcdFilterWindow, &PCDFilterWindow::signalClosePCDFilterWindow, this, &MainWindow::slotCloseOtherWindow);
    }
    pcdFilterWindow->show();
}

void MainWindow::slotBirdViewProcess()
{
    birdViewPorcess = new BirdViewProcess();
    birdViewPorcess->setModal(true);
    birdViewPorcess->exec();
    birdViewPorcess->deleteLater();
    birdViewPorcess = nullptr;
}

void MainWindow::slotCameraIntrinsics()
{
    cameraIntrinsicsWindow = new CameraIntrinsicsWindow();
    cameraIntrinsicsWindow->setModal(true);
    cameraIntrinsicsWindow->exec();
    cameraIntrinsicsWindow->deleteLater();
    cameraIntrinsicsWindow = nullptr;
}

void MainWindow::slotCameraVerification()
{
    cameraVerificationWindow = new CameraVerificationWindow();
    cameraVerificationWindow->setModal(true);
    cameraVerificationWindow->exec();
    cameraVerificationWindow->deleteLater();
    cameraVerificationWindow = nullptr;
}

void MainWindow::slotRadarCameraManual()
{
    radarCameraManualWindow = new RadarCameraManualWindow();
    radarCameraManualWindow->setModal(true);
    radarCameraManualWindow->exec();
    radarCameraManualWindow->deleteLater();
    radarCameraManualWindow = nullptr;
}

void MainWindow::slotLidarRadarManual()
{
    lidarRadarManualWindow = new LidarRadarManualWindow();
    lidarRadarManualWindow->setModal(true);
    lidarRadarManualWindow->exec();
    lidarRadarManualWindow->deleteLater();
    lidarRadarManualWindow = nullptr;
}

void MainWindow::slotAbout()
{
    QMessageBox::about(this, tr("样本标注系统"), tr("样本标注系统 版本 %1").arg(qApp->applicationVersion()));
}

void MainWindow::slotUserManual()
{
    QDesktopServices::openUrl(QUrl(tr(":/docs/docs/user_manuals.pdf"), QUrl::TolerantMode));
}

void MainWindow::slotSelectMarkShape(const QString &text)
{
    int index = this->shapeBox->currentData().toInt();
    switch (loadDataType)
    {
    case MarkDataType::IMAGE:
        markWindow[MarkDataType::IMAGE]->setDrawShape(index);
        break;
    case MarkDataType::VIDEO:
        markWindow[MarkDataType::VIDEO]->setDrawShape(index);
        break;
    case MarkDataType::SEGMENT:
        markWindow[MarkDataType::SEGMENT]->setDrawShape(index);
        break;
    case MarkDataType::OCR:
        markWindow[MarkDataType::OCR]->setDrawShape(index);
        break;
    case MarkDataType::IMAGE_TRACKING:
        markWindow[MarkDataType::IMAGE_TRACKING]->setDrawShape(index);
        break;
    case MarkDataType::PCD:
        break;
    case MarkDataType::UNKNOWN:
        break;
    case MarkDataType::MAX_CONUT:
        break;
    }
}

void MainWindow::slotIsMarkStatus(bool isMark)
{
    if(isMark)
    {
        manualParamterAction->setEnabled(false);
        segmentParamterAction->setEnabled(false);
#if EDGE_TOOL == 0
        autoParamterAction->setEnabled(false);
        videoMarkParamterAction->setEnabled(false);
        pointcloudParamterAction->setEnabled(false);
#endif
    }
    else
    {
        manualParamterAction->setEnabled(true);
        segmentParamterAction->setEnabled(true);
#if EDGE_TOOL == 0
        autoParamterAction->setEnabled(true);
        videoMarkParamterAction->setEnabled(true);
        pointcloudParamterAction->setEnabled(true);
#endif
    }
}

void MainWindow::slotCloseOtherWindow(QString flag)
{
    if(flag.contains("mark"))
    {
        disconnect(autoDetection2DWindow, &AutoDetection2DWindow::signalCloseAutoSampleMarkWindow, this, &MainWindow::slotCloseOtherWindow);
        autoDetection2DWindow->deleteLater();;
        autoDetection2DWindow = nullptr;
    }
    else if(flag.contains("segLabelConverter"))
    {
        disconnect(segLabelConvertWindow, &SegmentationLabelConvertWindow::signalCloseSegLabelConverterWindow, this, &MainWindow::slotCloseOtherWindow);
        segLabelConvertWindow->deleteLater();
        segLabelConvertWindow = nullptr;
    }
    else if(flag.contains("videoToPicture"))
    {
        disconnect(videoToPictureWindow, &FromVideoToPictureWindow::signalCloseVideoToPictureWindow, this, &MainWindow::slotCloseOtherWindow);
        videoToPictureWindow->deleteLater();
        videoToPictureWindow = nullptr;
    }
    else if(flag.contains("videoFromPicture"))
    {
        disconnect(videoFromPictureWindow, &FromPictureToVideoWindow::signalCloseVideoFromPictureWindow, this, &MainWindow::slotCloseOtherWindow);
        videoFromPictureWindow->deleteLater();
        videoFromPictureWindow = nullptr;
    }
    else if(flag.contains("videoCropping"))
    {
        disconnect(videoCroppingWindow, &VideoCroppingWindow::signalCloseVideoCroppingWindow, this, &MainWindow::slotCloseOtherWindow);
        videoCroppingWindow->deleteLater();
        videoCroppingWindow = nullptr;
    }
    else if(flag.contains("videoCutting"))
    {
        disconnect(videoCuttingWindow, &VideoCuttingWindow::signalCloseVideoCuttingWindow, this, &MainWindow::slotCloseOtherWindow);
        videoCuttingWindow->deleteLater();
        videoCuttingWindow = nullptr;
    }
    else if(flag.contains("imageConverter"))
    {
        disconnect(imageConverterWindow, &ImageConverterWindow::signalCloseImageConverterWindow, this, &MainWindow::slotCloseOtherWindow);
        imageConverterWindow->deleteLater();
        imageConverterWindow = nullptr;
    }
    else if(flag.contains("camera"))
    {
        disconnect(cameraWindow, &QCameraWindow::signalCloseCameraWindow, this, &MainWindow::slotCloseOtherWindow);
        cameraWindow->deleteLater();
        cameraWindow = nullptr;
    }
    else if(flag.contains("pcdConverter"))
    {
        disconnect(pcdConverterWindow, &PCDConverterWindow::signalClosePCDConverterWindow, this, &MainWindow::slotCloseOtherWindow);
        pcdConverterWindow->deleteLater();
        pcdConverterWindow = nullptr;
    }
    else if(flag.contains("pcdFilter"))
    {
        disconnect(pcdFilterWindow, &PCDFilterWindow::signalClosePCDFilterWindow, this, &MainWindow::slotCloseOtherWindow);
        pcdFilterWindow->deleteLater();
        pcdFilterWindow = nullptr;
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if(autoDetection2DWindow != nullptr)
        autoDetection2DWindow->close();
    if(segLabelConvertWindow != nullptr)
        segLabelConvertWindow->close();
    if(videoToPictureWindow != nullptr)
        videoToPictureWindow->close();
    if(videoFromPictureWindow != nullptr)
        videoFromPictureWindow->close();
    if(videoCroppingWindow != nullptr)
        videoCroppingWindow->close();
    if(videoCuttingWindow != nullptr)
        videoCuttingWindow->close();
    if(imageConverterWindow != nullptr)
        imageConverterWindow->close();
    if(cameraWindow != nullptr)
        cameraWindow->close();
    if(pcdConverterWindow != nullptr)
        pcdConverterWindow->close();
    if(pcdFilterWindow != nullptr)
        pcdFilterWindow->close();
    if(birdViewPorcess != nullptr)
        birdViewPorcess->close();
    if(cameraIntrinsicsWindow != nullptr)
        cameraIntrinsicsWindow->close();
    if(centerWidget != nullptr)
    {
        centerWidget->currentWidget()->close();
        centerWidget->close();
    }
    QMainWindow::closeEvent(event);
}

void MainWindow::initData()
{
    autoDetection2DWindow = nullptr;
    segLabelConvertWindow = nullptr;
    videoToPictureWindow = nullptr;
    videoFromPictureWindow = nullptr;
    videoCuttingWindow = nullptr;
    videoCroppingWindow = nullptr;
    imageConverterWindow = nullptr;
    cameraWindow = nullptr;

    pcdConverterWindow = nullptr;
    pcdFilterWindow = nullptr;

    birdViewPorcess = nullptr;
    cameraIntrinsicsWindow = nullptr;
    cameraVerificationWindow = nullptr;
    radarCameraManualWindow = nullptr;
    lidarRadarManualWindow = nullptr;

    openDataDir = ".";
    loadDataType = MarkDataType::UNKNOWN;
}

void MainWindow::initAction()
{
    //file
    openImageDirAction = new QAction(tr("打开图片文件夹"), this);
    openImageDirAction->setIcon(QIcon(tr(":/images/images/open.png")));
    openSegmentImageDirAction = new QAction(tr("打开分割图片文件夹"), this);
    openSegmentImageDirAction->setIcon(QIcon(tr(":/images/images/segment.png")));
    openOCRImageDirAction = new QAction(tr("打开字符图片文件夹"), this);
    openOCRImageDirAction->setIcon(QIcon(tr(":/images/images/ocr.png")));
    openImageTrackingDirAction = new QAction(tr("打开追踪图片文件夹"), this);
    openImageTrackingDirAction->setIcon(QIcon(tr(":/images/images/open.png")));
    openVideoDirAction = new QAction(tr("打开视频文件夹"), this);
    openVideoDirAction->setIcon(QIcon(tr(":/images/images/video.png")));
    openPCDDirAction = new QAction(tr("打开点云文件夹"), this);
    openPCDDirAction->setIcon(QIcon(tr(":/images/images/pcl.png")));
    exitAction = new QAction(tr("退出系统"), this);
    //setting
    manualParamterAction = new QAction(tr("手动标注参数设置"), this);
    segmentParamterAction = new QAction(tr("分割标注参数设置"), this);
    autoParamterAction = new QAction(tr("自动标注参数设置"), this);
    videoMarkParamterAction = new QAction(tr("视频标注参数设置"), this);
    pointcloudParamterAction = new QAction(tr("点云标注参数设置"));
    //autoMark
    autoDet2dAction = new QAction(tr("自动化检测样本标注"), this);
    autoDet2dAction->setIcon(QIcon(tr(":/images/images/mark.png")));
    //tool
    segLabelConvertAction = new QAction(tr("分割图生成"), this);
    segLabelConvertAction->setIcon(QIcon(tr(":/images/images/seg.png")));
    videoToPictureAction = new QAction(tr("视频转换为图片"), this);
    videoToPictureAction->setIcon(QIcon(tr(":/images/images/cut.png")));
    videoFromPictureAction = new QAction(tr("图片转换为视频"), this);
    videoFromPictureAction->setIcon(QIcon(tr(":/images/images/imageTovideo.png")));
    videoCuttingAction = new QAction(tr("视频剪切"), this);
    videoCuttingAction->setIcon(QIcon(tr(":/images/images/videoCut.png")));
    videoCroppingAction = new QAction(tr("视频剪辑"), this);
    videoCroppingAction->setIcon(QIcon(tr(":/images/images/crop.png")));
    imageConverterAction = new QAction(tr("图片格式转换"), this);
    imageConverterAction->setIcon(QIcon(tr(":/images/images/play.png")));
    cameraAction = new QAction(tr("视频采集"), this);
    cameraAction->setIcon(QIcon(tr(":/images/images/record.png")));

    pcdConverterAction = new QAction(tr("点云格式转换"), this);
    pcdConverterAction->setIcon(QIcon(tr(":/images/images/pcl.png")));

    pcdFilterAction = new QAction(tr("PCD文件过滤"), this);
    pcdFilterAction->setIcon(QIcon(tr(":/images/images/pcl.png")));

    birdViewCalibratAction = new QAction(tr("鸟瞰图标定"), this);
    cameraIntrinsicsAction = new QAction(tr("相机内参标定"), this);
    cameraVerificationAction = new QAction(tr("相机畸变验证"), this);
    radarCameraManualCalibratAction = new QAction(tr("radar-camera手动标定"), this);
    lidarRadarManualCalibratAction = new QAction(tr("lidar-radar手动标定"), this);

    //about
    aboutAction = new QAction(tr("关于"), this);
    userManualAction = new QAction(tr("系统说明"), this);

    //shapeTool

    shapeWidget = new QWidget(this);
    shapeLabel = new QLabel(tr("选择标注形状"));
    shapeBox = new QComboBox();
    shapeBox->clear();
    QHBoxLayout *shapLayout = new QHBoxLayout();
    shapLayout->setSpacing(10);
    shapLayout->addWidget(shapeLabel);
    shapLayout->addWidget(shapeBox);
    shapeWidget->setLayout(shapLayout);

}

void MainWindow::initMenuBar()
{
    initAction();
    //file
    fileMenu = new QMenu(tr("文件"), this);
    fileMenu->addAction(openImageDirAction);
    fileMenu->addAction(openSegmentImageDirAction);
    fileMenu->addAction(openOCRImageDirAction);
    fileMenu->addAction(openImageTrackingDirAction);
#if EDGE_TOOL == 0
    fileMenu->addAction(openVideoDirAction);
    fileMenu->addAction(openPCDDirAction);
#endif
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);
    //setting
    settingMenu = new QMenu(tr("设置"), this);
    settingMenu->addAction(manualParamterAction);
    settingMenu->addAction(segmentParamterAction);
#if EDGE_TOOL == 0
    settingMenu->addAction(videoMarkParamterAction);
    settingMenu->addSeparator();
    settingMenu->addAction(autoParamterAction);
    settingMenu->addSeparator();
    settingMenu->addAction(pointcloudParamterAction);
#endif
    //autoMark
    autoMarkMenu = new QMenu(tr("自动化标注"), this);
    autoMarkMenu->addAction(autoDet2dAction);
    //tool
    toolMenu = new QMenu(tr("工具"), this);
    toolMenu->addAction(segLabelConvertAction);
    toolMenu->addSeparator();
    toolMenu->addAction(videoToPictureAction);
    toolMenu->addAction(videoFromPictureAction);
    toolMenu->addAction(videoCuttingAction);
    toolMenu->addAction(videoCroppingAction);
    toolMenu->addAction(imageConverterAction);
#if EDGE_TOOL == 0
    toolMenu->addAction(cameraAction);
    toolMenu->addSeparator();
    toolMenu->addAction(pcdConverterAction);
    toolMenu->addAction(pcdFilterAction);
#endif
    // calibration
    calibrationMenu = new QMenu(tr("传感器标定"), this);
    calibrationMenu->addAction(birdViewCalibratAction);
    calibrationMenu->addAction(cameraIntrinsicsAction);
    calibrationMenu->addAction(cameraVerificationAction);
    calibrationMenu->addAction(radarCameraManualCalibratAction);
    calibrationMenu->addAction(lidarRadarManualCalibratAction);
    //about
    aboutMenu = new QMenu(tr("关于"), this);
    aboutMenu->addAction(aboutAction);
    aboutMenu->addAction(userManualAction);

    this->menuBar()->addMenu(fileMenu);
    this->menuBar()->addMenu(settingMenu);
#if EDGE_TOOL == 0
    this->menuBar()->addMenu(autoMarkMenu);
#endif
    this->menuBar()->addMenu(toolMenu);
    this->menuBar()->addMenu(calibrationMenu);
    this->menuBar()->addMenu(aboutMenu);

    this->statusBar()->hide();
}

void MainWindow::initToolBar()
{
    //file
    fileTool = new QToolBar(tr("文件"));
    fileTool->setIconSize(QSize(30, 30));
    fileTool->addAction(openImageDirAction);
    fileTool->addAction(openSegmentImageDirAction);
    fileTool->addAction(openOCRImageDirAction);
    fileTool->addAction(openImageTrackingDirAction);
#if EDGE_TOOL == 0
    fileTool->addAction(openVideoDirAction);
    fileTool->addAction(openPCDDirAction);
#endif
    //autoMark
    autoMarkTool = new QToolBar(tr("自动化标注"));
    autoMarkTool->setIconSize(QSize(30, 30));
    autoMarkTool->addAction(autoDet2dAction);
    //shapeTool
    shapeTool = new QToolBar(tr("标注形状"));
    shapeTool->addWidget(shapeWidget);

    this->addToolBar(fileTool);
#if EDGE_TOOL == 0
    this->addToolBar(autoMarkTool);
#endif
    this->addToolBar(shapeTool);
}

void MainWindow::initUI()
{
   centerWidget = new QStackedWidget(this);

   markWindow.clear();
   markWindow.append(new ControlWindow(this));
   markWindow.append(new ImageControlWindow(this));
   markWindow.append(new VideoControlWindow(this));
   markWindow.append(new ImageSegmentControlWindow(this));
   markWindow.append(new OCRControlWindow(this));
   markWindow.append(new ImageTrackingControlWindow(this));
   markWindow.append(new PCLControlWindow(this));

   for(int loop = 0; loop < markWindow.size(); loop++)
   {
       centerWidget->addWidget(markWindow[loop]);
   }
   centerWidget->setCurrentIndex(loadDataType);

   this->setCentralWidget(centerWidget);
   //this->setMaximumSize(700,520);
   this->setMinimumSize(1100, 700);
   this->setWindowTitle(tr("样本标注系统"));
}

void MainWindow::initConnect()
{
    //file
    connect(openImageDirAction, &QAction::triggered, this, &MainWindow::slotOpenImageDir);
    connect(openSegmentImageDirAction, &QAction::triggered, this, &MainWindow::slotOpenImageSegmentDir);
    connect(openOCRImageDirAction, &QAction::triggered, this, &MainWindow::slotOpenOCRImageDir);
    connect(openImageTrackingDirAction, &QAction::triggered, this, &MainWindow::slotOpenImageTrackingDir);
    connect(openVideoDirAction, &QAction::triggered, this, &MainWindow::slotOpenVideoDir);
    connect(openPCDDirAction, &QAction::triggered, this, &MainWindow::slotOpenPCDDir);
    connect(exitAction, &QAction::triggered, this, &MainWindow::close);
    //setting
    connect(manualParamterAction, &QAction::triggered, this, &MainWindow::slotManualMarkParamterConfig);
    connect(segmentParamterAction, &QAction::triggered, this, &MainWindow::slotSegmentMarkParamterConfig);
    connect(autoParamterAction, &QAction::triggered, this, &MainWindow::slotAutoMarkParamterConfig);
    connect(videoMarkParamterAction, &QAction::triggered, this, &MainWindow::slotVideoMarkParamterConfig);
    connect(pointcloudParamterAction, &QAction::triggered, this, &MainWindow::slotPointCloudParamterConfig);
    //autoMark
    connect(autoDet2dAction, &QAction::triggered, this, &MainWindow::slotAutoSampleMark);
    //tool
    connect(segLabelConvertAction, &QAction::triggered, this, &MainWindow::slotSegLabelConvert);

    connect(videoToPictureAction, &QAction::triggered, this, &MainWindow::slotVideoToPicture);
    connect(videoFromPictureAction, &QAction::triggered, this, &MainWindow::slotVideoFromPicture);
    connect(videoCuttingAction, &QAction::triggered, this, &MainWindow::slotVideoCutting);
    connect(videoCroppingAction, &QAction::triggered, this, &MainWindow::slotVideoCropping);
    connect(imageConverterAction, &QAction::triggered, this, &MainWindow::slotImageConverter);
    connect(cameraAction, &QAction::triggered, this, &MainWindow::slotCamera);

    connect(pcdConverterAction, &QAction::triggered, this, &MainWindow::slotPcdConverter);
    connect(pcdFilterAction, &QAction::triggered, this, &MainWindow::slotPcdFilter);

    // calibration
    connect(birdViewCalibratAction, &QAction::triggered, this, &MainWindow::slotBirdViewProcess);
    connect(cameraIntrinsicsAction, &QAction::triggered, this, &MainWindow::slotCameraIntrinsics);
    connect(cameraVerificationAction, &QAction::triggered, this, &MainWindow::slotCameraVerification);
    connect(radarCameraManualCalibratAction, &QAction::triggered, this, &MainWindow::slotRadarCameraManual);
    connect(lidarRadarManualCalibratAction, &QAction::triggered, this, &MainWindow::slotLidarRadarManual);

    //about
    connect(aboutAction, &QAction::triggered, this, &MainWindow::slotAbout);
    connect(userManualAction, &QAction::triggered, this, &MainWindow::slotUserManual);

    connect(shapeBox, &QComboBox::currentTextChanged, this, &MainWindow::slotSelectMarkShape);

    for(int loop = 0; loop < markWindow.size(); loop++)
    {
        connect(this, &MainWindow::signalManualMarkParamterChanged, markWindow[loop], &ControlWindow::slotManualMarkParamterChanged);
        connect(markWindow[loop], &ControlWindow::signalIsMarkChanged, this, &MainWindow::slotIsMarkStatus);
    }
}

void MainWindow::initImageMarkShape()
{
    QMap<int, QString> shapeDatas = imgShape.getImageShape();
    shapeBox->clear();
    for(QMap<int, QString>::const_iterator iter = shapeDatas.constBegin();
        iter != shapeDatas.constEnd(); ++iter)
    {
        shapeBox->addItem(iter.value(), iter.key());
    }
}

void MainWindow::initSegmentMarkShape()
{
    QMap<int, QString> shapeDatas = imgShape.getSegmentShape();
    shapeBox->clear();
    for(QMap<int, QString>::const_iterator iter = shapeDatas.constBegin();
        iter != shapeDatas.constEnd(); ++iter)
    {
        shapeBox->addItem(iter.value(), iter.key());
    }
}

void MainWindow::initOCRMarkShape()
{
    QMap<int, QString> shapeDatas = imgShape.getOCRShape();
    shapeBox->clear();
    for(QMap<int, QString>::const_iterator iter = shapeDatas.constBegin();
        iter != shapeDatas.constEnd(); ++iter)
    {
        shapeBox->addItem(iter.value(), iter.key());
    }
}

void MainWindow::initImageTrackingMarkShape()
{
    QMap<int, QString> shapeDatas = imgShape.getTracking2dShape();
    shapeBox->clear();
    for(QMap<int, QString>::const_iterator iter = shapeDatas.constBegin();
        iter != shapeDatas.constEnd(); ++iter)
    {
        shapeBox->addItem(iter.value(), iter.key());
    }
}

void MainWindow::initPointCloudMarkShape()
{
    shapeBox->clear();
}
