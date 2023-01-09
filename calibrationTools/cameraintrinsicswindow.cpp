#ifdef WIN32
#pragma execution_character_set("utf-8")
#endif
#include "cameraintrinsicswindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>
#include <QDebug>
#include "helpers/dirprocess.h"

CameraIntrinsicsWindow::CameraIntrinsicsWindow(QDialog *parent) : QDialog(parent)
{
    init();
    initUI();
    initConnect();
}

CameraIntrinsicsWindow::~CameraIntrinsicsWindow()
{

}

void CameraIntrinsicsWindow::slotOpenImageDir()
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
        calibrationButton->setEnabled(false);
        undistortButton->setEnabled(false);
        saveResultButton->setEnabled(false);
    }
}

void CameraIntrinsicsWindow::slotImageItem(QListWidgetItem *item)
{
    QString imagePath = item->text();
    if(currentImage.load(imagePath))
    {
        int index = imageListWidget->row(item);
        currentImagePath = imagePath;
        cv::Mat image = calibrationProcess.getDrawCornerImage(currentImagePath.toStdString(), index);
        if(image.empty())
        {
            imageShow->setNewQImage(currentImage);
            calibrationButton->setEnabled(true);
            undistortButton->setEnabled(false);
            saveResultButton->setEnabled(false);
        }
        else
        {
            cv::Mat rgbFrame;
            cv::cvtColor(image, rgbFrame, cv::COLOR_BGR2RGB);
            currentImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
            imageShow->setNewQImage(currentImage);
        }
    }
    else
    {
        currentImagePath = "";
        QMessageBox::information(this, tr("加载图片"), tr("该图片加载失败,请点击下一张图片！"));
    }
}

void CameraIntrinsicsWindow::slotCalibrate()
{
    QDir makeDir;
    cv::Size image_size(currentImage.width(), currentImage.height());
    cv::Size board_size(boardSizeWidthBox->value(), boardSizeHeightBox->value());
    cv::Size square_size(squareSizeWidthBox->value(), squareSizeHeightBox->value());
    QString saveDir = this->openDataDir + "/result";
    if(!makeDir.exists(saveDir))
    {
        if(!makeDir.mkdir(saveDir))
        {
            qDebug() << saveDir << " fail!" << endl;
        }
    }
    std::string err_result;
    std::vector<std::string> images_list;
    calibrationProcess.setInitData(cameraModelBox->currentData().toInt(), \
                                   image_size, board_size, square_size, \
                                   saveDir.toStdString());
    for(int index = 0; index < processDataList.size(); index++)
    {
        images_list.push_back(processDataList[index].toStdString());
    }
    if(calibrationProcess.calibrating(images_list, err_result))
    {
        this->commandText->append(QString::fromStdString(err_result));
        if(calibrationProcess.getSuccessImage() < MIN_CALI_IMAGE_NUM)
        {
            QMessageBox::StandardButton button;
            button = QMessageBox::question(this, tr("Calibration"), tr("可以检测到角点的图像（%1张）少于 %2！ 是否增加图片重新标定").arg(calibrationProcess.getSuccessImage())
                                         .arg(MIN_CALI_IMAGE_NUM), QMessageBox::Yes|QMessageBox::No);
            if(button==QMessageBox::No)
            {
                undistortButton->setEnabled(true);
                saveResultButton->setEnabled(true);
            }
            else if(button==QMessageBox::Yes)
            {
                calibrationButton->setEnabled(false);
                undistortButton->setEnabled(false);
                saveResultButton->setEnabled(false);
            }
        }
        else
        {
            undistortButton->setEnabled(true);
            saveResultButton->setEnabled(true);
        }
    }
    else
    {
        QMessageBox::information(this, tr("calibration"), tr("可以检测到角点的图像太少（%1张）！").arg(calibrationProcess.getSuccessImage()));
        undistortButton->setEnabled(false);
        saveResultButton->setEnabled(false);
    }
}

void CameraIntrinsicsWindow::slotShowUndistortImage()
{
    if(currentImagePath != "")
    {
        cv::Mat rgbFrame;
        cv::Mat image = calibrationProcess.getUndistortImage(currentImagePath.toStdString());
        cv::cvtColor(image, rgbFrame, cv::COLOR_BGR2RGB);
        currentImage = QImage((uchar*)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, QImage::Format_RGB888);
        imageShow->setNewQImage(currentImage);
    }
}

void CameraIntrinsicsWindow::slotSaveCalibrateResult()
{
    calibrationProcess.saveCalibrationResult();
    if(this->isSaveUndistortBox->isChecked())
    {
        std::vector<std::string> images_list;
        for(int index = 0; index < processDataList.size(); index++)
        {
            images_list.push_back(processDataList[index].toStdString());
        }
        calibrationProcess.saveUndistortImage(images_list);
    }
    if(this->isSelectCornersImageBox->isChecked())
    {
        std::vector<std::string> images_list;
        for(int index = 0; index < processDataList.size(); index++)
        {
            images_list.push_back(processDataList[index].toStdString());
        }
        calibrationProcess.saveDrawCornerImage(images_list);
    }
    this->commandText->append(tr("Save result success!"));
}

void CameraIntrinsicsWindow::init()
{
    this->currentImage = QImage(tr(":/images/images/play.png"));
    currentImagePath = "";
    openDataDir = ".";
    processDataList.clear();
}

void CameraIntrinsicsWindow::initUI()
{
    QHBoxLayout *boardLayout = new QHBoxLayout();
    boardSizeLabel = new QLabel(tr("棋盘格角点数:"));
    boardSizeWidthBox = new QSpinBox();
    boardSizeWidthBox->setValue(11);
    boardSizeWidthBox->setMinimum(1);
    boardSizeWidthBox->setSingleStep(1);
    boardSizeWidthBox->setPrefix("w:");
    boardSizeHeightBox = new QSpinBox();
    boardSizeHeightBox->setValue(8);
    boardSizeHeightBox->setMinimum(1);
    boardSizeHeightBox->setSingleStep(1);
    boardSizeHeightBox->setPrefix("h:");
    boardLayout->setSpacing(10);
    boardLayout->addWidget(boardSizeLabel);
    boardLayout->addWidget(boardSizeWidthBox);
    boardLayout->addWidget(boardSizeHeightBox);

    QHBoxLayout *squareLayout = new QHBoxLayout();
    squareSizeLabel = new QLabel(tr("棋盘格尺寸:"));
    squareSizeWidthBox = new QSpinBox();
    squareSizeWidthBox->setValue(20);
    squareSizeWidthBox->setMinimum(1);
    squareSizeWidthBox->setSingleStep(1);
    squareSizeWidthBox->setPrefix("w:");
    squareSizeWidthBox->setSuffix("mm");
    squareSizeHeightBox = new QSpinBox();
    squareSizeHeightBox->setValue(20);
    squareSizeHeightBox->setMinimum(1);
    squareSizeHeightBox->setSingleStep(1);
    squareSizeHeightBox->setPrefix("h:");
    squareSizeHeightBox->setSuffix("mm");
    squareLayout->setSpacing(10);
    squareLayout->addWidget(squareSizeLabel);
    squareLayout->addWidget(squareSizeWidthBox);
    squareLayout->addWidget(squareSizeHeightBox);

    QHBoxLayout *cameraModelLayout = new QHBoxLayout();
    cameraModelLabel = new QLabel(tr("相机模型:"));
    cameraModelBox = new QComboBox();
    cameraModelBox->addItem(tr("Pinhole"), 1);
    cameraModelBox->addItem(tr("Fisheye"), 2);
    cameraModelLayout->setSpacing(5);
    cameraModelLayout->addWidget(cameraModelLabel);
    cameraModelLayout->addWidget(cameraModelBox);

    isSaveUndistortBox = new QCheckBox(tr("是否保存反畸变图像"));
    isSelectCornersImageBox = new QCheckBox(tr("是否保存角点图像"));
    QHBoxLayout *savelLayout = new QHBoxLayout();
    savelLayout->addWidget(isSaveUndistortBox);
    savelLayout->addWidget(isSelectCornersImageBox);

    QGridLayout *centerTopLayout = new QGridLayout();
    centerTopLayout->setSpacing(20);
    centerTopLayout->addLayout(boardLayout, 0, 0, 1, 1);
    centerTopLayout->addLayout(squareLayout, 1, 0, 1, 1);
    centerTopLayout->addLayout(cameraModelLayout, 0, 1, 1, 1);
    centerTopLayout->addLayout(savelLayout, 1, 1, 1, 1);
    paramGroupBox = new QGroupBox(tr("参数设置"));
    paramGroupBox->setLayout(centerTopLayout);

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
    centerLayout->addWidget(paramGroupBox);
    centerLayout->addWidget(scrollArea);
    centerLayout->addWidget(commandText);
    centerLayout->setStretchFactor(paramGroupBox, 1);
    centerLayout->setStretchFactor(scrollArea, 5);
    centerLayout->setStretchFactor(commandText, 1);

    imageListWidget = new QListWidget();

    openDirButton = new QPushButton(tr("打开图片文件夹"));
    calibrationButton = new QPushButton(tr("启动标定"));
    undistortButton = new QPushButton(tr("图片反畸变"));
    saveResultButton = new QPushButton(tr("保存标定结果"));
    QVBoxLayout *buttonLayout = new QVBoxLayout();
    buttonLayout->setSpacing(10);
    buttonLayout->addWidget(openDirButton);
    buttonLayout->addWidget(calibrationButton);
    buttonLayout->addWidget(undistortButton);
    buttonLayout->addWidget(saveResultButton);
    calibrationButton->setEnabled(false);
    undistortButton->setEnabled(false);
    saveResultButton->setEnabled(false);

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
    this->setWindowTitle(tr("相机内参标定"));
}

void CameraIntrinsicsWindow::initConnect()
{
    connect(openDirButton, &QPushButton::clicked, this, &CameraIntrinsicsWindow::slotOpenImageDir);
    connect(imageListWidget, &QListWidget::itemClicked, this, &CameraIntrinsicsWindow::slotImageItem);
    connect(calibrationButton, &QPushButton::clicked, this, &CameraIntrinsicsWindow::slotCalibrate);
    connect(undistortButton, &QPushButton::clicked, this, &CameraIntrinsicsWindow::slotShowUndistortImage);
    connect(saveResultButton, &QPushButton::clicked, this, &CameraIntrinsicsWindow::slotSaveCalibrateResult);
}
