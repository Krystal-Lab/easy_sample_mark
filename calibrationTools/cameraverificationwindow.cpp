#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "cameraverificationwindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>
#include <QDebug>
#include "helpers/dirprocess.h"

CameraVerificationWindow::CameraVerificationWindow(QDialog *parent) : QDialog(parent)
{
    init();
    initUI();
    initConnect();
}

CameraVerificationWindow::~CameraVerificationWindow()
{

}

void CameraVerificationWindow::slotOpenImageDir()
{
    DirProcess dirProcess;
    processDataList.clear();
    this->openDataDir = QFileDialog::getExistingDirectory(this, tr("选择文件夹"), openDataDir, QFileDialog::ShowDirsOnly);
    if(this->openDataDir.trimmed().isEmpty() || !QDir(this->openDataDir).exists())
    {
        qDebug() << "打开的文件路径有误:" << this->openDataDir << endl;
    }
    else
    {
        QStringList filter;
        filter << "*.png" << "*.jpg" << "*.jpeg" << "*.bmp";
        processDataList = dirProcess.getDirFileName(this->openDataDir, filter);
        imageListWidget->clear();
        for(int index = 0; index < processDataList.size(); index++)
        {
            QListWidgetItem *item = new QListWidgetItem(QIcon(":/qss_icons/style/rc/checkbox_checked_focus.png"),
                                                        processDataList[index]);
            item->setData(Qt::UserRole, 1);
            imageListWidget->insertItem(index, item);
        }
        verificationButton->setEnabled(false);
    }
}

void CameraVerificationWindow::slotImageItem(QListWidgetItem *item)
{
    QString imagePath = item->text();
    if(currentImage.load(imagePath))
    {
        currentImagePath = imagePath;
        imageShow->setNewQImage(currentImage);
        verificationButton->setEnabled(true);
        saveButton->setEnabled(false);
    }
    else
    {
        currentImagePath = "";
        verificationButton->setEnabled(false);
        saveButton->setEnabled(false);
        QMessageBox::information(this, tr("加载图片"), tr("该图片加载失败,请点击下一张图片！"));
    }
}

void CameraVerificationWindow::slotVerification()
{
//    double d, d_max;
//    if(currentImagePath != "")
//    {
//        std::cout << "currentImagePath:" << currentImagePath.toStdString() << std::endl;
//        distortionMeasurement.measure(currentImagePath.toStdString(), d, d_max);
//        this->commandText->append(tr("畸变验证："));
//        this->commandText->append(tr("d: %1").arg(d));
//        this->commandText->append(tr("d_max: %1").arg(d_max));
//        saveButton->setEnabled(true);
//    }
    if(currentImagePath != "")
    {
        if(undistortModelBox->currentData().toInt() == 1)
        {
            calibrationProcess.setCameraParam(cameraModelBox->currentData().toInt(), cameraInstrinsics, distortionCoefficients);
            cv::Mat image = calibrationProcess.getUndistortImage(currentImagePath.toStdString(), scale_focal, shift_center);
            cv::cvtColor(image, rgbFrame, cv::COLOR_BGR2RGB);
            currentImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
        }
        else if(undistortModelBox->currentData().toInt() == 2 && cameraModelBox->currentData().toInt() == 2)
        {
            cv::Mat input_mat = cv::imread(currentImagePath.toStdString());
            cv::Mat image = fisheyeCameraProcess.transverseCorrection(input_mat);
            cv::cvtColor(image, rgbFrame, cv::COLOR_BGR2RGB);
            currentImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
        }
        else if(undistortModelBox->currentData().toInt() == 3 && cameraModelBox->currentData().toInt() == 2)
        {
            cv::Mat input_mat = cv::imread(currentImagePath.toStdString());
            cv::Mat image = fisheyeCameraProcess.longitudeCorrection(input_mat);
            cv::cvtColor(image, rgbFrame, cv::COLOR_BGR2RGB);
            currentImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
        }
        else if(undistortModelBox->currentData().toInt() == 4 && cameraModelBox->currentData().toInt() == 2)
        {
            cv::Mat input_mat = cv::imread(currentImagePath.toStdString());
            cv::Mat image = fisheyeCameraProcess.latitudeCorrection(input_mat);
            cv::cvtColor(image, rgbFrame, cv::COLOR_BGR2RGB);
            currentImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
        }
        else if(undistortModelBox->currentData().toInt() == 5 && cameraModelBox->currentData().toInt() == 2)
        {
            cv::Mat input_mat = cv::imread(currentImagePath.toStdString());
            cv::Mat image = fisheyeCameraProcess.dewarpCorrection(input_mat);
            cv::cvtColor(image, rgbFrame, cv::COLOR_BGR2RGB);
            currentImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
        }
        else
        {
            QMessageBox::information(this, tr("畸变验证"), tr("参数选择有误！"));
            currentImage.load(currentImagePath);
        }
        imageShow->setNewQImage(currentImage);
    }

}

void CameraVerificationWindow::slotSaveImage()
{
    if(currentImagePath != "" && !currentImage.isNull())
    {
        QDir makeDir;
        QFileInfo imageFileInfo(currentImagePath);
        QString saveDir = openDataDir + "/" + "temp";
        if(!makeDir.exists(saveDir))
        {
            if(!makeDir.mkdir(saveDir))
            {
                qDebug() << "make Annotations dir fail!" << endl;
            }
        }
        QString savePath = saveDir + "/" + imageFileInfo.completeBaseName() + ".png";
        currentImage.save(savePath);
    }
}

void CameraVerificationWindow::slotLoadCameraIntrinsic()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Param File"),
                                                    openDataDir,
                                                    tr("Files (*.json)"));
    if(fileName.trimmed().isEmpty())
        return;
    QFileInfo imageFileInfo(fileName);
    openDataDir = imageFileInfo.path();
    if(paramLoad.loadCameraIntrinsic(fileName, cameraInstrinsics, distortionCoefficients, scale_focal, shift_center))
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
        verificationButton->setEnabled(false);
        saveButton->setEnabled(false);
    }
}

void CameraVerificationWindow::init()
{
    this->currentImage = QImage(tr(":/images/images/play.png"));
    currentImagePath = "";
    openDataDir = ".";
    processDataList.clear();

    isLoadIntrinsic = false;
    cameraInstrinsics = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    distortionCoefficients = cv::Mat(5, 1,CV_32FC1, cv::Scalar::all(0));
    scale_focal = cv::Point2f(1, 1);
    shift_center = cv::Point2f(0, 0);
}

void CameraVerificationWindow::initUI()
{
    QHBoxLayout *intrinsicLayout = new QHBoxLayout();
    intrinsicText = new QLineEdit();
    intrinsicText->setReadOnly(true);
    openIntrinsicButton = new QPushButton(tr("加载相机内参文件"));
    intrinsicLayout->setSpacing(20);
    intrinsicLayout->addWidget(intrinsicText);
    intrinsicLayout->addWidget(openIntrinsicButton);

    QHBoxLayout *cameraModelLayout = new QHBoxLayout();
    cameraModelLabel = new QLabel(tr("相机模型:"));
    cameraModelBox = new QComboBox();
    cameraModelBox->addItem(tr("Pinhole"), 1);
    cameraModelBox->addItem(tr("Fisheye"), 2);
    cameraModelLayout->setSpacing(5);
    cameraModelLayout->addWidget(cameraModelLabel);
    cameraModelLayout->addWidget(cameraModelBox);

    QHBoxLayout *undistortModelLayout = new QHBoxLayout();
    undistortModelLabel = new QLabel(tr("反畸变方法:"));
    undistortModelBox = new QComboBox();
    undistortModelBox->addItem(tr("Checkerboard Correction"), 1);
    undistortModelBox->addItem(tr("Transverse Correction"), 2);
    undistortModelBox->addItem(tr("Longitude Correction"), 3);
    undistortModelBox->addItem(tr("Latitude Correction"), 4);
    undistortModelBox->addItem(tr("Longitude and Latitude"), 5);
    undistortModelLayout->setSpacing(5);
    undistortModelLayout->addWidget(undistortModelLabel);
    undistortModelLayout->addWidget(undistortModelBox);

    QHBoxLayout *topLayout = new QHBoxLayout();
    topLayout->setSpacing(10);
    topLayout->addLayout(cameraModelLayout);
    topLayout->addLayout(intrinsicLayout);
    topLayout->addLayout(undistortModelLayout);

    imageShow = new ImageLabel();
    imageShow->setNewQImage(currentImage);
    imageShow->setEnabled(false);
    scrollArea = new QScrollArea();
    scrollArea->setAlignment(Qt::AlignCenter);
    scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->setAutoFillBackground(true);
    //scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);  //控件大小 小于 视窗大小时，默认不会显示滚动条
    //scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);    //强制显示滚动条。
    scrollArea->setWidget(imageShow);

    commandText = new MyTextBrowser();
    //commandText->setFixedHeight(100);
    commandText->setReadOnly(true);

    QVBoxLayout *centerLayout = new QVBoxLayout();
    centerLayout->setSpacing(10);
    centerLayout->addLayout(topLayout);
    centerLayout->addWidget(scrollArea);
    centerLayout->addWidget(commandText);
    centerLayout->setStretchFactor(topLayout, 1);
    centerLayout->setStretchFactor(scrollArea, 5);
    centerLayout->setStretchFactor(commandText, 1);

    imageListWidget = new QListWidget();

    openDirButton = new QPushButton(tr("打开图片文件夹"));
    verificationButton = new QPushButton(tr("矫正图像"));
    saveButton =new QPushButton(tr("保存图片"));
    QVBoxLayout *buttonLayout = new QVBoxLayout();
    buttonLayout->setSpacing(10);
    buttonLayout->addWidget(openDirButton);
    buttonLayout->addWidget(verificationButton);
    buttonLayout->addWidget(saveButton);
    verificationButton->setEnabled(false);
    saveButton->setEnabled(false);

    QVBoxLayout *rightLayout = new QVBoxLayout();
    rightLayout->addWidget(imageListWidget);
    rightLayout->addLayout(buttonLayout);
    rightLayout->setStretchFactor(imageListWidget, 5);
    rightLayout->setStretchFactor(buttonLayout, 1);

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->setMargin(10); //设置这个对话框的边距
    mainLayout->setSpacing(10);  //设置各个控件之间的边距
    mainLayout->setAlignment(Qt::AlignCenter);
    mainLayout->addLayout(centerLayout);
    mainLayout->addLayout(rightLayout);
    mainLayout->setStretchFactor(centerLayout, 4);
    mainLayout->setStretchFactor(rightLayout, 1);
    this->setLayout(mainLayout);
    //this->setMaximumSize(700,520);
    this->setMinimumSize(1000, 600);
    this->setWindowTitle(tr("相机畸变验证"));
}

void CameraVerificationWindow::initConnect()
{
    connect(openDirButton, &QPushButton::clicked, this, &CameraVerificationWindow::slotOpenImageDir);
    connect(imageListWidget, &QListWidget::itemClicked, this, &CameraVerificationWindow::slotImageItem);
    connect(verificationButton, &QPushButton::clicked, this, &CameraVerificationWindow::slotVerification);
    connect(saveButton, &QPushButton::clicked, this, &CameraVerificationWindow::slotSaveImage);
    connect(openIntrinsicButton, &QPushButton::clicked, this, &CameraVerificationWindow::slotLoadCameraIntrinsic);
}
