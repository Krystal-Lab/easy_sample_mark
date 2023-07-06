#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "radarcameramanualwindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QFileInfo>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QDebug>
#include <sstream>
#include <fstream>
#include <iostream>

RadarCameraManualWindow::RadarCameraManualWindow(QDialog *parent) : QDialog(parent)
{
    init();
    initUI();
    initConnect();
}

RadarCameraManualWindow::~RadarCameraManualWindow()
{

}

void RadarCameraManualWindow::slotLoadIntrinsic()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Param File"),
                                                    openDataDir,
                                                    tr("Files (*.json)"));
    if(fileName.trimmed().isEmpty())
        return;
    QFileInfo imageFileInfo(fileName);
    openDataDir = imageFileInfo.path();
    if(paramLoad.loadCameraIntrinsic(fileName, cameraInstrinsics, distortionCoefficients))
    {
        intrinsicText->setText(fileName);
        std::ostringstream tempStr;
        tempStr << "Intrinsic:\n" << cameraInstrinsics << "\n";
        tempStr << "Distortion:\n" << distortionCoefficients << "\n";
        this->commandText->append(QString::fromStdString(tempStr.str()));
        isLoadIntrinsic = true;
    }
    else
    {
        isLoadIntrinsic = false;
        intrinsicText->setText("");
        QMessageBox::information(this, tr("加载相机内参"), tr("加载相机内参失败！"));
    }
    updateButtonStatus();
}

void RadarCameraManualWindow::slotLoadHomography()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Param File"),
                                                    openDataDir,
                                                    tr("Files (*.json)"));
    if(fileName.trimmed().isEmpty())
        return;
    QFileInfo imageFileInfo(fileName);
    openDataDir = imageFileInfo.path();
    if(paramLoad.loadCameraHomography(fileName, homography))
    {
        homographyText->setText(fileName);
        std::ostringstream tempStr;
        tempStr << "Homography:\n" << homography << "\n";
        this->commandText->append(QString::fromStdString(tempStr.str()));
        isLoadHomography = true;
    }
    else
    {
        isLoadHomography = false;
        homographyText->setText("");
        QMessageBox::information(this, tr("加载相机Homography"), tr("加载相机Homography参数失败！"));
    }
    updateButtonStatus();
}

void RadarCameraManualWindow::slotLoadExtrinsic()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Param File"),
                                                    openDataDir,
                                                    tr("Files (*.json)"));
    if(fileName.trimmed().isEmpty())
        return;
    QFileInfo imageFileInfo(fileName);
    openDataDir = imageFileInfo.path();
    if(loadExtrinsic(fileName))
    {
        extrinsicText->setText(fileName);
        std::ostringstream tempStr;
        tempStr << "Extrinsic:\n"
            << orign_calibration_matrix_ << "\n";
        this->commandText->append(QString::fromStdString(tempStr.str()));
        isLoadExtrinsic = true;
    }
    else
    {
        isLoadExtrinsic = false;
        extrinsicText->setText("");
        QMessageBox::information(this, tr("加载Radar-camera外参"), tr("加载Radar-camera外参失败！"));
    }
    updateButtonStatus();
}

void RadarCameraManualWindow::slotLoadImage()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Image File"),
                                                    openDataDir,
                                                    tr("Images (*.png *.jpg *.jpeg)"));
    if(fileName.trimmed().isEmpty())
        return;
    QFileInfo imageFileInfo(fileName);
    openDataDir = imageFileInfo.path();
    if(currentImage.load(fileName))
    {
        QImage image = currentImage.convertToFormat(QImage::Format_RGB888);
        currentMat = convertImage.QImageTocvMat(image).clone();
        imageShow->setNewQImage(currentImage);
        openRadarButton->setEnabled(true);
        this->commandText->append(tr("加载图像数据成功!"));
    }
    else
    {
        openRadarButton->setEnabled(false);
        QMessageBox::information(this, tr("加载图片"), tr("该图片加载失败！"));
    }
}

void RadarCameraManualWindow::slotLoadRadar()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Radar File"),
                                                    openDataDir,
                                                    tr("Files (*.csv)"));
    if(fileName.trimmed().isEmpty())
        return;
    QFileInfo imageFileInfo(fileName);
    openDataDir = imageFileInfo.path();
    if(loadRadarData(fileName))
    {
        selectPointButton->setEnabled(true);
        this->commandText->append(tr("加载Radar数据成功!"));
    }
    else
    {
        selectPointButton->setEnabled(false);
        QMessageBox::information(this, tr("加载Radar数据"), tr("加载Radar数据失败！"));
    }
}

void RadarCameraManualWindow::slotSelectPoint()
{
    isInit = false;
    selectPointList.clear();
    showCalibrationButton->setEnabled(false);
    paramGroundBox->setEnabled(false);
    if(selectPointButton->text() == QString(tr("开始选择Lane点")))
    {
        imageShow->setNewQImage(currentImage);
        imageShow->setEnabled(true);
        selectPointButton->setText(tr("完成选择Lane点"));
    }
    else if(selectPointButton->text() == QString(tr("完成选择Lane点")))
    {
        selectPointList = imageShow->getPointList();
        if(selectPointList.count() == 4)
        {
            std::ostringstream tempStr;
            tempStr << "Select Point:\n";
            std::vector<cv::Point2f> picked_points;
            for(int i = 0; i < selectPointList.count(); i++)
            {
                cv::Point2f point(selectPointList[i].x(), selectPointList[i].y());
                picked_points.push_back(point);
                tempStr << selectPointList[i].x() << "," << selectPointList[i].y() << "\n";
            }
            this->commandText->append(QString::fromStdString(tempStr.str()));
            int width = currentMat.cols;
            int height = currentMat.rows;
            int bv_width = width / 2;
            int bv_height = height;
            projector.init(currentMat, homography, bv_width, bv_height, picked_points);
            projector.loadPointCloud(point3DList);

            showCalibrationButton->setEnabled(true);
            paramGroundBox->setEnabled(true);

            calibrationInit();
        }
        else
        {
            QMessageBox::information(this, tr("选择点"), tr("选择点数必须等于4, 请重新选择点！"));
        }
        imageShow->setEnabled(false);
        selectPointButton->setText(tr("开始选择Lane点"));
    }
}

void RadarCameraManualWindow::slotShowCalibration()
{
    if(isInit)
    {
        cv::Mat current_frame;
        cv::Mat rgbFrame;
        cv::Mat bv_frame;
        cv::Mat rgb_bv;
        Eigen::Matrix4f temp = calibration_matrix_ * modification_matrix_;
        Transform transform;
        transform.setTransform(temp);
        projector.ProjectImage(currentMat, cameraInstrinsics,
                               distortionCoefficients, transform,
                               current_frame);
        projector.ProjectBirdView(transform, bv_frame);
    //    cv::imwrite("image.jpg", current_frame);
    //    cv::imwrite("bv_image.jpg", bv_frame);
        cv::cvtColor(current_frame, rgbFrame, cv::COLOR_BGR2RGB);
        QImage frameImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
        cv::cvtColor(bv_frame, rgb_bv, cv::COLOR_BGR2RGB);
        QImage bvImage = QImage((uchar*)rgb_bv.data, rgb_bv.cols, rgb_bv.rows, QImage::Format_RGB888);
        imageShow->setNewQImage(frameImage);
        birdImageShow->setNewQImage(bvImage);
        imageShow->setScaleImage(scaleShowBox->value());
        birdImageShow->setScaleImage(scaleShowBox->value());
    }
}

void RadarCameraManualWindow::slotStepChange(double value)
{
    qDebug() << "step:" << value << endl;
    xDegreeBox->setSingleStep(scaleDegreeBox->value());
    yDegreeBox->setSingleStep(scaleDegreeBox->value());
    zDegreeBox->setSingleStep(scaleDegreeBox->value());

    xTransBox->setSingleStep(scaleTransBox->value());
    yTransBox->setSingleStep(scaleTransBox->value());
    zTransBox->setSingleStep(scaleTransBox->value());
}

void RadarCameraManualWindow::slotDegreeParamChange(double value)
{
    Eigen::Matrix4f temp;
    Eigen::Matrix3f rot_tmp;
    rot_tmp =
            Eigen::AngleAxisf(xDegreeBox->value() / 180.0 * M_PI,
                              Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(yDegreeBox->value() / 180.0 * M_PI,
                              Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(zDegreeBox->value() / 180.0 * M_PI,
                              Eigen::Vector3f::UnitY());
    modification_matrix_.block(0, 0, 3, 3) = rot_tmp;

    temp = calibration_matrix_ * modification_matrix_;
    Transform transform;
    transform.setTransform(temp);
    Transform::Vector6 T_log = transform.log();
    // std::cout << "T_log:" << T_log << std::endl;
    QString xStr = QString::number(T_log(3) * 180 / M_PI, 'f', 3);
    xDegreeLabel->setText(tr("X Degrage: %1").arg(xStr));
    QString yStr = QString::number(T_log(4) * 180 / M_PI, 'f', 3);
    yDegreeLabel->setText(tr("Y Degrage: %1").arg(yStr));
    QString zStr = QString::number(T_log(5) * 180 / M_PI, 'f', 3);
    zDegreeLabel->setText(tr("Z Degrage: %1").arg(zStr));

    slotShowCalibration();
}

void RadarCameraManualWindow::slotTransParamChange(double value)
{
    qDebug() << " T param:" << value << endl;
    if(isInit)
    {
        calibration_matrix_(0, 3) = xTransBox->value();
        calibration_matrix_(1, 3) = yTransBox->value();
        calibration_matrix_(2, 3) = zTransBox->value();
        slotShowCalibration();
    }
}

void RadarCameraManualWindow::slotScaleImageShow(int value)
{
    this->imageShow->setScaleImage(value);
    this->birdImageShow->setScaleImage(value);
}

void RadarCameraManualWindow::slotResetCalibration()
{
    calibrationInit();
    slotShowCalibration();
}

void RadarCameraManualWindow::slotSaveResult()
{
    std::ostringstream tempStr;
    Eigen::Matrix4f temp = calibration_matrix_ * modification_matrix_;
    Transform transform;
    transform.setTransform(temp);
    tempStr << "Extrinsic:" << std::endl;
    Transform::Rotation R = transform.rotation();
    Transform::Vector6 T_log = transform.log();
    tempStr << "Matrix:\n";
    tempStr << temp << std::endl;
    tempStr << "Quaternion:\n";
    tempStr << R.w() << " " << R.x() << " " << R.y() << " " << R.z() << std::endl;
    tempStr << "Angle:\n";
    tempStr << T_log[3] << " " << T_log[4] << " " << T_log[5] << std::endl;
    this->commandText->append(QString::fromStdString(tempStr.str()));
}

void RadarCameraManualWindow::init()
{
    openDataDir = ".";
    isLoadIntrinsic = false;
    isLoadHomography = false;
    isLoadExtrinsic = false;
    isInit = false;
    this->currentImage = QImage(tr(":/images/images/play.png"));
    this->birdViewImage = QImage(tr(":/images/images/play.png"));
    cameraInstrinsics = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    distortionCoefficients = cv::Mat(5, 1,CV_32FC1, cv::Scalar::all(0));
    homography = cv::Mat(3, 3, CV_32F, cv::Scalar::all (0));
    calibration_matrix_.setIdentity();
    orign_calibration_matrix_.setIdentity();
    modification_matrix_.setIdentity();
    point3DList.clear();
    selectPointList.clear();
}

void RadarCameraManualWindow::initUI()
{
    intrinsicText = new QLineEdit();
    intrinsicText->setReadOnly(true);
    openIntrinsicButton = new QPushButton(tr("加载相机内参文件"));
    homographyText = new QLineEdit();
    homographyText->setReadOnly(true);
    openHomographyButton = new QPushButton(tr("加载相机homography文件"));
    extrinsicText = new QLineEdit();
    extrinsicText->setReadOnly(true);
    openExtrinsicButton = new QPushButton(tr("加载radar2camera外参文件"));

    openImageButton = new QPushButton(tr("加载标定图像文件"));
    openRadarButton = new QPushButton(tr("加载标定Radar文件"));
    selectPointButton = new QPushButton(tr("开始选择Lane点"));
    showCalibrationButton = new QPushButton(tr("显示标定效果"));
    updateButtonStatus();

    QGridLayout *topLeftLayout = new QGridLayout();
    topLeftLayout->setSpacing(10);
    topLeftLayout->addWidget(intrinsicText, 0, 0, 1, 3);
    topLeftLayout->addWidget(openIntrinsicButton, 0, 3);
    topLeftLayout->addWidget(homographyText, 1, 0, 1, 3);
    topLeftLayout->addWidget(openHomographyButton, 1, 3);
    topLeftLayout->addWidget(extrinsicText, 2, 0, 1, 3);
    topLeftLayout->addWidget(openExtrinsicButton, 2, 3);
    topLeftLayout->addWidget(openImageButton, 3, 0);
    topLeftLayout->addWidget(openRadarButton, 3, 1);
    topLeftLayout->addWidget(selectPointButton, 3, 2);
    topLeftLayout->addWidget(showCalibrationButton, 3, 3);
    //设置行列比例系数
//    topLeftLayout->setRowStretch(0, 3);
//    topLeftLayout->setRowStretch(0, 3);
//    topLeftLayout->setRowStretch(2, 3);
//    topLeftLayout->setRowStretch(3, 1);
//    topLeftLayout->setRowStretch(4, 1);
//    topLeftLayout->setColumnStretch(0, 3);
//    topLeftLayout->setColumnStretch(1, 3);
//    topLeftLayout->setColumnStretch(2, 3);
//    topLeftLayout->setColumnStretch(3, 1);
//    topLeftLayout->setColumnStretch(4, 1);

    scaleDegreeLabel = new QLabel(tr("Degrage Step:"));
    scaleDegreeBox = new QDoubleSpinBox();
    scaleDegreeBox->setValue(0.1);
    scaleDegreeBox->setMaximum(360);
    scaleDegreeBox->setMinimum(-360);
    scaleDegreeBox->setSingleStep(0.1);
    scaleDegreeBox->setSuffix(tr("度"));
    scaleDegreeBox->setDecimals(2);
    scaleDegreeBox->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    scaleTransLabel = new QLabel(tr("Translation Step："));
    scaleTransBox = new QDoubleSpinBox();
    scaleTransBox->setValue(0.060);
    scaleTransBox->setMaximum(1000);
    scaleTransBox->setMinimum(-1000);
    scaleTransBox->setSingleStep(0.001);
    scaleTransBox->setSuffix("m");
    scaleTransBox->setDecimals(3);
    scaleTransBox->setFocusPolicy(Qt::FocusPolicy::NoFocus);

    QHBoxLayout *paramLayout1 = new QHBoxLayout();
    paramLayout1->setSpacing(10);
    paramLayout1->addWidget(scaleDegreeLabel);
    paramLayout1->addWidget(scaleDegreeBox);
    paramLayout1->addWidget(scaleTransLabel);
    paramLayout1->addWidget(scaleTransBox);

    xDegreeLabel = new QLabel(tr("X Degrage:"));
    xDegreeBox = new QDoubleSpinBox();
    xDegreeBox->setValue(0);
    xDegreeBox->setMaximum(360);
    xDegreeBox->setMinimum(-360);
    xDegreeBox->setSingleStep(scaleDegreeBox->value());
    xDegreeBox->setSuffix(tr("度"));
    xDegreeBox->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    yDegreeLabel = new QLabel(tr("Y Degrage:"));
    yDegreeBox = new QDoubleSpinBox();
    yDegreeBox->setValue(0);
    yDegreeBox->setMaximum(360);
    yDegreeBox->setMinimum(-360);
    yDegreeBox->setSingleStep(scaleDegreeBox->value());
    yDegreeBox->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    yDegreeBox->setSuffix(tr("度"));
    zDegreeLabel = new QLabel(tr("X Degrage:"));
    zDegreeBox = new QDoubleSpinBox();
    zDegreeBox->setValue(0);
    zDegreeBox->setMaximum(360);
    zDegreeBox->setMinimum(-360);
    zDegreeBox->setSingleStep(scaleDegreeBox->value());
    zDegreeBox->setSuffix(tr("度"));
    zDegreeBox->setFocusPolicy(Qt::FocusPolicy::NoFocus);

    QHBoxLayout *paramLayout2 = new QHBoxLayout();
    paramLayout2->setSpacing(10);
    paramLayout2->addWidget(xDegreeLabel);
    paramLayout2->addWidget(xDegreeBox);
    paramLayout2->addWidget(yDegreeLabel);
    paramLayout2->addWidget(yDegreeBox);
    paramLayout2->addWidget(zDegreeLabel);
    paramLayout2->addWidget(zDegreeBox);

    xTransLabel = new QLabel(tr("X Translation:"));
    xTransBox = new QDoubleSpinBox();
    xTransBox->setValue(0);
    xTransBox->setMaximum(1000);
    xTransBox->setMinimum(-1000);
    xTransBox->setSingleStep(scaleTransBox->value());
    xTransBox->setSuffix("m");
    xTransBox->setDecimals(3);
    xTransBox->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    yTransLabel = new QLabel(tr("Y Translation:"));
    yTransBox = new QDoubleSpinBox();
    yTransBox->setValue(0);
    yTransBox->setMaximum(1000);
    yTransBox->setMinimum(-1000);
    yTransBox->setSingleStep(scaleTransBox->value());
    yTransBox->setSuffix("m");
    yTransBox->setDecimals(3);
    yTransBox->setFocusPolicy(Qt::FocusPolicy::NoFocus);
    zTransLabel = new QLabel(tr("Z Translation:"));
    zTransBox = new QDoubleSpinBox();
    zTransBox->setValue(0);
    zTransBox->setMaximum(1000);
    zTransBox->setMinimum(-1000);
    zTransBox->setSingleStep(scaleTransBox->value());
    zTransBox->setSuffix("m");
    zTransBox->setDecimals(3);
    zTransBox->setFocusPolicy(Qt::FocusPolicy::NoFocus);

    QHBoxLayout *paramLayout3 = new QHBoxLayout();
    paramLayout3->setSpacing(10);
    paramLayout3->addWidget(xTransLabel);
    paramLayout3->addWidget(xTransBox);
    paramLayout3->addWidget(yTransLabel);
    paramLayout3->addWidget(yTransBox);
    paramLayout3->addWidget(zTransLabel);
    paramLayout3->addWidget(zTransBox);

    scaleShowLabel = new QLabel(tr("缩放图片:"));
    scaleShowBox = new QSlider(Qt::Horizontal, this);
    scaleShowBox->move(10, 200);
    scaleShowBox->setMinimum(10); //设置最小值
    scaleShowBox->setMaximum(200);  //设置最大值
    scaleShowBox->setValue(100); //设置滑动条控件的值
    QHBoxLayout *scaleLayout = new QHBoxLayout();
    scaleLayout->setSpacing(5);
    scaleLayout->addWidget(scaleShowLabel);
    scaleLayout->addWidget(scaleShowBox);

    resetButton = new QPushButton(tr("重置标定参数"));
    saveResultButton = new QPushButton(tr("保存标定参数"));
    QHBoxLayout *paramLayout4 = new QHBoxLayout();
    paramLayout4->setSpacing(20);
    paramLayout4->addLayout(scaleLayout);
    paramLayout4->addWidget(resetButton);
    paramLayout4->addWidget(saveResultButton);

    QVBoxLayout *paramLayout = new QVBoxLayout();
    // paramLayout->setSpacing(20);
    paramLayout->addLayout(paramLayout1);
    paramLayout->addLayout(paramLayout2);
    paramLayout->addLayout(paramLayout3);
    paramLayout->addLayout(paramLayout4);
    paramGroundBox = new QGroupBox(tr("参数"));
    paramGroundBox->setLayout(paramLayout);
    paramGroundBox->setEnabled(false);

    QHBoxLayout *topLayout = new QHBoxLayout();
    topLayout->setSpacing(10);
    topLayout->addLayout(topLeftLayout);
    topLayout->addWidget(paramGroundBox);

    imageShow = new ImageLabel();
    imageShow->setNewQImage(currentImage);
    imageShow->setEnabled(false);
    imageScrollArea = new QScrollArea();
    imageScrollArea->setAlignment(Qt::AlignCenter);
    imageScrollArea->setBackgroundRole(QPalette::Dark);
    imageScrollArea->setAutoFillBackground(true);
    //scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);  //控件大小 小于 视窗大小时，默认不会显示滚动条
    //scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);    //强制显示滚动条。
    imageScrollArea->setWidget(imageShow);

    birdImageShow = new ImageLabel();
    birdImageShow->setNewQImage(birdViewImage);
    birdImageShow->setEnabled(false);
    birdScrollArea = new QScrollArea();
    birdScrollArea->setAlignment(Qt::AlignCenter);
    birdScrollArea->setBackgroundRole(QPalette::Dark);
    birdScrollArea->setAutoFillBackground(true);
    //birdScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);  //控件大小 小于 视窗大小时，默认不会显示滚动条
    //birdScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);    //强制显示滚动条。
    birdScrollArea->setWidget(birdImageShow);

    QHBoxLayout *centerLayout = new QHBoxLayout();
    centerLayout->setSpacing(10);
    centerLayout->addWidget(imageScrollArea);
    centerLayout->addWidget(birdScrollArea);
    centerLayout->setStretchFactor(imageScrollArea, 2);
    centerLayout->setStretchFactor(birdScrollArea, 1);

    commandText = new MyTextBrowser();
    //commandText->setFixedHeight(100);
    commandText->setReadOnly(true);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setMargin(10); //设置这个对话框的边距
    mainLayout->setSpacing(10);  //设置各个控件之间的边距
    mainLayout->setAlignment(Qt::AlignCenter);
    mainLayout->addLayout(topLayout);
    mainLayout->addLayout(centerLayout);
    mainLayout->addWidget(commandText);
    mainLayout->setStretchFactor(topLayout, 1);
    mainLayout->setStretchFactor(centerLayout, 5);
    mainLayout->setStretchFactor(commandText, 1);
    this->setLayout(mainLayout);
    //this->setMaximumSize(700,520);
    this->setMinimumSize(1250, 800);
    this->setWindowTitle(tr("radar-camera手动标定"));
}

void RadarCameraManualWindow::initConnect()
{
    connect(scaleDegreeBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RadarCameraManualWindow::slotStepChange);
    connect(scaleTransBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RadarCameraManualWindow::slotStepChange);

    connect(openIntrinsicButton, &QPushButton::clicked, this, &RadarCameraManualWindow::slotLoadIntrinsic);
    connect(openHomographyButton, &QPushButton::clicked, this, &RadarCameraManualWindow::slotLoadHomography);
    connect(openExtrinsicButton, &QPushButton::clicked, this, &RadarCameraManualWindow::slotLoadExtrinsic);
    connect(openImageButton, &QPushButton::clicked, this, &RadarCameraManualWindow::slotLoadImage);
    connect(openRadarButton, &QPushButton::clicked, this, &RadarCameraManualWindow::slotLoadRadar);
    connect(selectPointButton, &QPushButton::clicked, this, &RadarCameraManualWindow::slotSelectPoint);
    connect(showCalibrationButton, &QPushButton::clicked, this, &RadarCameraManualWindow::slotShowCalibration);
    connect(resetButton, &QPushButton::clicked, this, &RadarCameraManualWindow::slotResetCalibration);
    connect(saveResultButton, &QPushButton::clicked, this, &RadarCameraManualWindow::slotSaveResult);

    connect(xDegreeBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RadarCameraManualWindow::slotDegreeParamChange);
    connect(yDegreeBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RadarCameraManualWindow::slotDegreeParamChange);
    connect(zDegreeBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RadarCameraManualWindow::slotDegreeParamChange);
    connect(xTransBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RadarCameraManualWindow::slotTransParamChange);
    connect(yTransBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RadarCameraManualWindow::slotTransParamChange);
    connect(zTransBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &RadarCameraManualWindow::slotTransParamChange);

    connect(scaleShowBox, &QSlider::valueChanged, this, &RadarCameraManualWindow::slotScaleImageShow);
}

void RadarCameraManualWindow::updateButtonStatus()
{
    if(isLoadIntrinsic && isLoadHomography && isLoadExtrinsic)
    {
        openImageButton->setEnabled(true);
        openRadarButton->setEnabled(false);
        selectPointButton->setEnabled(false);
        showCalibrationButton->setEnabled(false);
    }
    else
    {
        openImageButton->setEnabled(false);
        openRadarButton->setEnabled(false);
        selectPointButton->setEnabled(false);
        showCalibrationButton->setEnabled(false);
    }
}

void RadarCameraManualWindow::calibrationInit()
{
//    Eigen::Matrix3f R = orign_calibration_matrix_.topLeftCorner<3, 3>();
//    std::cout << R << std::endl;
//    auto angle = R.eulerAngles(0, 1, 2) * 180 / M_PI;
//    auto angleypr = R.eulerAngles(2, 1, 0) * 180 / M_PI;
//    std::cout << "X  = " << angle(0) << std::endl;
//    std::cout << "Y  = " << angle(1) << std::endl;
//    std::cout << "Z  = " << angle(2) << std::endl;

//    std::cout << "yaw  = " << angleypr(0) << std::endl;
//    std::cout << "pitch  = " << angleypr(1) << std::endl;
//    std::cout << "roll  = " << angleypr(2) << std::endl;
    Transform transform;
    transform.setTransform(orign_calibration_matrix_);
//    Eigen::AngleAxisf angle_axis(transform.rotation());
//    std::cout << "angle:" << angle_axis.angle() << std::endl;
//    std::cout << "axis:" << angle_axis.axis() << std::endl;
    Transform::Vector6 T_log = transform.log();
    // std::cout << "T_log:" << T_log << std::endl;
    QString xStr = QString::number(T_log(3) * 180 / M_PI, 'f', 3);
    xDegreeLabel->setText(tr("X Degrage: %1").arg(xStr));
    QString yStr = QString::number(T_log(4) * 180 / M_PI, 'f', 3);
    yDegreeLabel->setText(tr("Y Degrage: %1").arg(yStr));
    QString zStr = QString::number(T_log(5) * 180 / M_PI, 'f', 3);
    zDegreeLabel->setText(tr("Z Degrage: %1").arg(zStr));

    xTransBox->setValue(T_log(0));
    yTransBox->setValue(T_log(1));
    zTransBox->setValue(T_log(2));
    xDegreeBox->setValue(0);
    yDegreeBox->setValue(0);
    zDegreeBox->setValue(0);
    scaleShowBox->setValue(100);
    calibration_matrix_ = orign_calibration_matrix_;
    isInit = true;
}

bool RadarCameraManualWindow::loadExtrinsic(const QString &filePath)
{
    bool result = false;
    QByteArray data;
    QFile file;
    file.setFileName(filePath);
    if(!file.open(QFile::ReadOnly | QFile::Text))
    {
        return result;
    }
    data = file.readAll();
    file.close();
    QJsonParseError jsonError;
    QJsonDocument parseDoucment = QJsonDocument::fromJson(QString(data).toUtf8(), &jsonError);

    if(jsonError.error == QJsonParseError::NoError)
    {
        if (!(parseDoucment.isNull() || parseDoucment.isEmpty()))
        {
            if (parseDoucment.isObject())
            {
                QJsonObject jsonObject = parseDoucment.object();
                if(jsonObject.contains("radar-camera-extrinsic"))
                {
                    QJsonObject rootObject = jsonObject.take("radar-camera-extrinsic").toObject();
                    if(rootObject.contains("param"))
                    {
                        QJsonObject paramObject = rootObject.take("param").toObject();
                        if(paramObject.contains("sensor_calib"))
                        {
                            QJsonObject extrinsicObject = paramObject.take("sensor_calib").toObject();
                            QJsonArray dataList = extrinsicObject.take("data").toArray();
                            for(int index = 0; index < dataList.size(); index++)
                            {
                                QJsonArray rowData = dataList.at(index).toArray();
                                orign_calibration_matrix_(index, 0) = static_cast<float>(rowData.at(0).toDouble());
                                orign_calibration_matrix_(index, 1) = static_cast<float>(rowData.at(1).toDouble());
                                orign_calibration_matrix_(index, 2) = static_cast<float>(rowData.at(2).toDouble());
                                orign_calibration_matrix_(index, 3) = static_cast<float>(rowData.at(3).toDouble());
                            }
                            result = true;
                        }
                    }
                }
            }
        }
    }
    else
    {
        qDebug() << "error:" << jsonError.errorString() << endl;
        return result;
    }
    return result;
}

bool RadarCameraManualWindow::loadRadarData(const QString &filePath)
{
    point3DList.clear();
    std::ifstream file(filePath.toStdString());
    if (!file.is_open())
    {
        std::cout << "ERROR--->>> cannot open: " << filePath.toStdString() << std::endl;
        return false;
    }
    std::string line;
    getline(file, line);
    // conti radar: the input point is the x and y coordinate
    bool whether_first = true;
    std::string first_time_str;
    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string str;
        std::string time_str;
        std::string position_x_str;
        std::string position_y_str;
        int index = 0;
        while (getline(ss, str, ','))
        {
            if (index == 0) {
                time_str = str;
                if (whether_first) {
                    first_time_str = str;
                    whether_first = false;
                } else {
                    long long gap = std::stoll(time_str) - std::stoll(first_time_str);
                    if (gap > 15 * 1e6) {
                        std::cout << "radar point size: " << point3DList.size() << std::endl;
                        return true;
                    }
                }
            }
            if (index == 4) {
                position_x_str = str;
            } else if (index == 5) {
                position_y_str = str;
            }
            index++;
        }

        cv::Point3f radar_point;
        radar_point.x = std::atof(position_x_str.c_str());
        radar_point.y = std::atof(position_y_str.c_str());
        radar_point.z = 0;
        if (std::abs(radar_point.x) < 1e-6 || std::abs(radar_point.y) < 1e-6)
        {
            continue;
        }
        point3DList.push_back(radar_point);
    }

    if(point3DList.size() > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
