#pragma once

#include <QDir>
#include <QImage>

#include <opencv2/opencv.hpp>

/**
 * 将OpenCV的cv::Mat转换为Qt的QImage
 * 支持常见的图像格式，包括灰度图、RGB/BGR和带Alpha通道的图像
 *
 * @param inMat 输入的OpenCV矩阵
 * @param copyData 是否强制复制数据（默认为false，使用共享数据）
 * @return 转换后的QImage对象
 */
QImage cvMatToQImage(const cv::Mat& inMat)
{
    if (inMat.type() == CV_8UC4) {
        // 8-bit, 4 channel
        QImage image(inMat.data,
            inMat.cols,
            inMat.rows,
            static_cast<int>(inMat.step),
            QImage::Format_ARGB32);
        return image;
    }
    else if (inMat.type() == CV_8UC3) {
        // 8-bit, 3 channel
        QImage image(inMat.data,
            inMat.cols,
            inMat.rows,
            static_cast<int>(inMat.step),
            QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if (inMat.type() == CV_8UC1) {
        // 8-bit, 1 channel
        QImage image(inMat.data,
            inMat.cols,
            inMat.rows,
            static_cast<int>(inMat.step),
            QImage::Format_Grayscale8);
        return image;
    }
    else {
		spdlog::warn("Unsupported cv::Mat type: {}", inMat.type());
        QImage* image = new QImage();
        return *image;
    }
}

/**
 * 获取日期作为文件名 保存图片
 */
void saveImage(const cv::Mat& img)
{
}

/**
 * 从指定目录加载图像文件并转换为cv::Mat列表
 * @param directoryPath 目录路径
 * @param images 输出的cv::Mat列表
 * @param supportedFormats 支持的图像格式列表
 * @return 成功加载的图像数量
 */
int loadImagesFromDirectory(const QString& directoryPath,
    std::vector<cv::Mat>& images,
    const QStringList& supportedFormats = QStringList()
    << "*.png" << "*.jpg" << "*.jpeg" << "*.bmp") {
    images.clear();

    // 检查目录是否存在
    QDir directory(directoryPath);
    if (!directory.exists()) {
		spdlog::warn("Directory does not exist: {}", directoryPath.toStdString());
        return 0;
    }

    // 设置名称过滤器以匹配图像文件
    directory.setNameFilters(supportedFormats);
    directory.setFilter(QDir::Files | QDir::Readable);

    // 获取所有匹配的文件
    QStringList imageFiles = directory.entryList();

    // 加载图像
    int loadedCount = 0;
    for (const QString& fileName : imageFiles) {
        QString filePath = directory.absoluteFilePath(fileName);
        cv::Mat image = cv::imread(filePath.toStdString(), cv::IMREAD_COLOR);

        if (!image.empty()) {
            images.push_back(image);
            ++loadedCount;
        }
        else {
			spdlog::warn("Failed to load image: {}", filePath.toStdString());
        }
    }

    return loadedCount;
}