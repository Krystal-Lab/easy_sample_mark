#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "lidarradarmanualwindow.h"
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

LidarRadarManualWindow::LidarRadarManualWindow(QDialog *parent) : QDialog(parent)
{
    init();
    initUI();
    initConnect();
}

LidarRadarManualWindow::~LidarRadarManualWindow()
{

}

void LidarRadarManualWindow::slotLoadPointCloud()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Param File"),
                                                    openDataDir,
                                                    tr("Files (*.pcd)"));
    if(fileName.trimmed().isEmpty())
        return;
    QFileInfo imageFileInfo(fileName);
    openDataDir = imageFileInfo.path();
    pointCloudText->setText(fileName);
    drawPointCloud->clearPoints();
    if(drawPointCloud->setNewPointCloud(fileName) != -1)
    {
        this->commandText->append(tr("加载Lidar数据成功!"));
        openPointCloudButton->setEnabled(true);
        openRadarButton->setEnabled(true);
        openExtrinsicButton->setEnabled(false);
        paramGroundBox->setEnabled(false);
    }
    else
    {
        openPointCloudButton->setEnabled(true);
        openRadarButton->setEnabled(false);
        openExtrinsicButton->setEnabled(false);
        paramGroundBox->setEnabled(false);
        QMessageBox::information(this, tr("加载pcd"), tr("加载pcd失败！"));
    }
}

void LidarRadarManualWindow::slotLoadRadar()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Radar File"),
                                                    openDataDir,
                                                    tr("Files (*.csv)"));
    if(fileName.trimmed().isEmpty())
        return;
    QFileInfo imageFileInfo(fileName);
    openDataDir = imageFileInfo.path();
    radarText->setText(fileName);
    if(radar_dataloader.loadRadarData(fileName, point3DList))
    {
        this->commandText->append(tr("加载Radar数据成功!"));
        openPointCloudButton->setEnabled(true);
        openRadarButton->setEnabled(true);
        openExtrinsicButton->setEnabled(true);
        paramGroundBox->setEnabled(false);
    }
    else
    {
        openPointCloudButton->setEnabled(true);
        openRadarButton->setEnabled(true);
        openExtrinsicButton->setEnabled(false);
        paramGroundBox->setEnabled(false);
        QMessageBox::information(this, tr("加载Radar数据"), tr("加载Radar数据失败！"));
    }
}

void LidarRadarManualWindow::slotLoadExtrinsic()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Param File"),
                                                    openDataDir,
                                                    tr("Files (*.json)"));
    if(fileName.trimmed().isEmpty())
        return;
    QFileInfo imageFileInfo(fileName);
    openDataDir = imageFileInfo.path();
    extrinsicText->setText(fileName);
    if(loadExtrinsic(fileName))
    {
        std::ostringstream tempStr;
        tempStr << "Extrinsic:\n"
            << orign_calibration_matrix_ << "\n";
        this->commandText->append(QString::fromStdString(tempStr.str()));
        paramGroundBox->setEnabled(true);
        calibrationInit();
        slotShowCalibration();
    }
    else
    {
        paramGroundBox->setEnabled(false);
        isInit = false;
        extrinsicText->setText("");
        QMessageBox::information(this, tr("加载Lidar-Radar外参"), tr("加载Lidar-Radar外参失败！"));
    }
}

void LidarRadarManualWindow::slotShowCalibration()
{
    if(isInit)
    {
        Eigen::Matrix4f temp = calibration_matrix_ * modification_matrix_;
        Transform transform;
        transform.setTransform(temp);
        pcl::PointCloud<pcl::PointXYZ>::Ptr radar_points(new pcl::PointCloud<pcl::PointXYZ>());
        for(size_t i = 0; i < point3DList.size(); i++)
        {
            Eigen::Vector3f object_center;
            object_center[0] = point3DList[i].x;
            object_center[1] = point3DList[i].y;
            object_center[2] = point3DList[i].z;
            object_center = transform.affine() * object_center;
            radar_points->push_back(pcl::PointXYZ(object_center[0], object_center[1], object_center[2]));
        }
        drawPointCloud->setRadarPoints(radar_points);
    }
}

void LidarRadarManualWindow::slotStepChange(double value)
{
    qDebug() << "step:" << value << endl;
    xDegreeBox->setSingleStep(scaleDegreeBox->value());
    yDegreeBox->setSingleStep(scaleDegreeBox->value());
    zDegreeBox->setSingleStep(scaleDegreeBox->value());

    xTransBox->setSingleStep(scaleTransBox->value());
    yTransBox->setSingleStep(scaleTransBox->value());
    zTransBox->setSingleStep(scaleTransBox->value());
}

void LidarRadarManualWindow::slotDegreeParamChange(double value)
{
    Eigen::Matrix4f temp;
    auto rot_tmp = Eigen::Matrix3f(
            Eigen::AngleAxisf(static_cast<float>(zDegreeBox->value() / 180.0 * M_PI), Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(static_cast<float>(yDegreeBox->value() / 180.0 * M_PI), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(static_cast<float>(xDegreeBox->value() / 180.0 * M_PI), Eigen::Vector3f::UnitX()));
    modification_matrix_.setIdentity();
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

void LidarRadarManualWindow::slotTransParamChange(double value)
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

void LidarRadarManualWindow::slotResetCalibration()
{
    calibrationInit();
    slotShowCalibration();
}

void LidarRadarManualWindow::slotSaveResult()
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

void LidarRadarManualWindow::init()
{
    openDataDir = ".";
    isInit = false;
    calibration_matrix_.setIdentity();
    orign_calibration_matrix_.setIdentity();
    modification_matrix_.setIdentity();
    point3DList.clear();
}

void LidarRadarManualWindow::initUI()
{
    pointCloudText = new QLineEdit();
    pointCloudText->setReadOnly(true);
    openPointCloudButton = new QPushButton(tr("加载点云数据"));
    radarText = new QLineEdit();
    radarText->setReadOnly(true);
    openRadarButton = new QPushButton(tr("加载Radar数据"));
    extrinsicText = new QLineEdit();
    extrinsicText->setReadOnly(true);
    openExtrinsicButton = new QPushButton(tr("加载radar2lidar外参文件"));

    openPointCloudButton->setEnabled(true);
    openRadarButton->setEnabled(false);
    openExtrinsicButton->setEnabled(false);

    QGridLayout *topLeftLayout = new QGridLayout();
    topLeftLayout->setSpacing(10);
    topLeftLayout->addWidget(pointCloudText, 0, 0, 1, 3);
    topLeftLayout->addWidget(openPointCloudButton, 0, 3);
    topLeftLayout->addWidget(radarText, 1, 0, 1, 3);
    topLeftLayout->addWidget(openRadarButton, 1, 3);
    topLeftLayout->addWidget(extrinsicText, 2, 0, 1, 3);
    topLeftLayout->addWidget(openExtrinsicButton, 2, 3);

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

    resetButton = new QPushButton(tr("重置标定参数"));
    saveResultButton = new QPushButton(tr("保存标定参数"));
    QHBoxLayout *paramLayout4 = new QHBoxLayout();
    paramLayout4->setSpacing(20);
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

    drawPointCloud = new PointCloudViewer(this);
    cloudScrollArea = new QScrollArea(this);
    cloudScrollArea->setAlignment(Qt::AlignCenter);
    cloudScrollArea->setWidgetResizable(true);
    cloudScrollArea->viewport()->setBackgroundRole(QPalette::Dark);
    cloudScrollArea->viewport()->setAutoFillBackground(true);
    //referenceScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);  //控件大小 小于 视窗大小时，默认不会显示滚动条
    //referenceScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);    //强制显示滚动条。
    cloudScrollArea->setWidget(drawPointCloud);


    commandText = new MyTextBrowser();
    //commandText->setFixedHeight(100);
    commandText->setReadOnly(true);

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setMargin(10); //设置这个对话框的边距
    mainLayout->setSpacing(10);  //设置各个控件之间的边距
    mainLayout->setAlignment(Qt::AlignCenter);
    mainLayout->addLayout(topLayout);
    mainLayout->addWidget(cloudScrollArea);
    mainLayout->addWidget(commandText);
    mainLayout->setStretchFactor(topLayout, 1);
    mainLayout->setStretchFactor(cloudScrollArea, 6);
    mainLayout->setStretchFactor(commandText, 1);
    this->setLayout(mainLayout);
    //this->setMaximumSize(700,520);
    this->setMinimumSize(1000, 800);
    this->setWindowTitle(tr("lidar-radar手动标定"));
}

void LidarRadarManualWindow::initConnect()
{
    connect(scaleDegreeBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &LidarRadarManualWindow::slotStepChange);
    connect(scaleTransBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &LidarRadarManualWindow::slotStepChange);

    connect(openPointCloudButton, &QPushButton::clicked, this, &LidarRadarManualWindow::slotLoadPointCloud);
    connect(openExtrinsicButton, &QPushButton::clicked, this, &LidarRadarManualWindow::slotLoadExtrinsic);
    connect(openRadarButton, &QPushButton::clicked, this, &LidarRadarManualWindow::slotLoadRadar);

    connect(resetButton, &QPushButton::clicked, this, &LidarRadarManualWindow::slotResetCalibration);
    connect(saveResultButton, &QPushButton::clicked, this, &LidarRadarManualWindow::slotSaveResult);

    connect(xDegreeBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &LidarRadarManualWindow::slotDegreeParamChange);
    connect(yDegreeBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &LidarRadarManualWindow::slotDegreeParamChange);
    connect(zDegreeBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &LidarRadarManualWindow::slotDegreeParamChange);
    connect(xTransBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &LidarRadarManualWindow::slotTransParamChange);
    connect(yTransBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &LidarRadarManualWindow::slotTransParamChange);
    connect(zTransBox, static_cast<void (QDoubleSpinBox ::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, &LidarRadarManualWindow::slotTransParamChange);
}

void LidarRadarManualWindow::calibrationInit()
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

    calibration_matrix_ = orign_calibration_matrix_;
    isInit = true;
}

bool LidarRadarManualWindow::loadExtrinsic(const QString &filePath)
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
                if(jsonObject.contains("radar-lidar-extrinsic"))
                {
                    QJsonObject rootObject = jsonObject.take("radar-lidar-extrinsic").toObject();
                    if(rootObject.contains("param"))
                    {
                        QJsonObject paramObject = rootObject.take("param").toObject();
                        if(paramObject.contains("translation") && paramObject.contains("rotation"))
                        {
                            QJsonArray translationList = paramObject.take("translation").toArray();
                            QJsonArray rotationList = paramObject.take("rotation").toArray();
                            if(translationList.size() == 3 && rotationList.size() == 4)
                            {
                                float t_x = static_cast<float>(translationList.at(0).toDouble());
                                float t_y = static_cast<float>(translationList.at(1).toDouble());
                                float t_z = static_cast<float>(translationList.at(2).toDouble());
                                float r_w = static_cast<float>(rotationList.at(0).toDouble());
                                float r_x = static_cast<float>(rotationList.at(1).toDouble());
                                float r_y = static_cast<float>(rotationList.at(2).toDouble());
                                float r_z = static_cast<float>(rotationList.at(3).toDouble());
                                Transform::Translation translation(t_x, t_y, t_z);
                                Transform::Rotation rotation(r_w, r_x, r_y, r_z);
                                transform_process = Transform(translation, rotation);
                                orign_calibration_matrix_ = transform_process.matrix();
                                result = true;
                            }
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
