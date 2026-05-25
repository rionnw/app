#include "LabelWidget.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>

#include <spdlog/spdlog.h>

LabelWidget::LabelWidget(QWidget *parent)
	: QWidget(parent)
{
	rois.clear();
	currentRoi = QRect(0, 0, 10, 10);

	image = QImage(1280, 960, QImage::Format_RGB32);

	setMouseTracking(true);
}
 
void LabelWidget::setImage(QImage& img)
{
	image = img;
	update();
}

void LabelWidget::setRois(const std::vector<cv::Rect>& cvRois)
{
	rois.clear();
	for (const auto& roi : cvRois) {
		this->rois.push_back(QRect(roi.x, roi.y, roi.width, roi.height));
	}
	currentRoi = QRect(0, 0, 10, 10);
	update();
}

std::vector<cv::Rect> LabelWidget::getRois() const
{
	std::vector<cv::Rect> cvRois;
	for (const auto& roi : rois) {
		cvRois.push_back(cv::Rect(roi.x(), roi.y(), roi.width(), roi.height()));
	}
	return cvRois;
}

void LabelWidget::resetRois()
{
	rois.clear();
	currentRoi = QRect(0, 0, 10, 10);
	update();
}

void LabelWidget::removeLastRoi()
{
	if (!rois.isEmpty()) {
		rois.removeLast();
		update();
	}
}

void LabelWidget::paintEvent(QPaintEvent* ev)
{
	Q_UNUSED(ev);
	QPainter painter(this);
	painter.translate(0, 0);

	painter.drawPixmap(QPoint(0, 0), QPixmap::fromImage(image));

	painter.setPen(Qt::black);
	painter.drawRect(currentRoi);

	for (int i = 0; i < rois.size(); ++i) {
		// 绘制ROI边框
		painter.setPen(roiColors[i % 3]);
		painter.drawRect(rois[i]);

		// 准备文字区域（ROI下方）
		QRect textRect(
			rois[i].x() - rois[i].width() / 4,
			rois[i].y() + rois[i].height() + 2,
			rois[i].width() * 1.5,
			16 // 文字高度
		);

		// 添加半透明背景（可选）
		painter.fillRect(textRect, QColor(0, 0, 0, 18));

		// 绘制白色文字
		painter.setPen(Qt::white);
		painter.drawText(textRect, Qt::AlignCenter, QString::number(i));
	}
}

void LabelWidget::mousePressEvent(QMouseEvent * ev)
{
	if (ev->button() == Qt::LeftButton) {
		// 1. 如果是左键点击，将当前 ROI 添加到 rois 列表
		rois.append(currentRoi);

		// 2. 可以添加视觉反馈，例如高亮显示选中的 ROI
		update();
	}
}

void LabelWidget::mouseMoveEvent(QMouseEvent* ev)
{
    auto xy = ev->pos();
    currentRoi.moveCenter(xy);
	update();
}

void LabelWidget::wheelEvent(QWheelEvent* ev)
{
	const int minSize = 5;
	const int maxSize = 100;

	// 获取滚轮角度变化（单位：1/8度）
	QPoint angleDelta = ev->angleDelta();
	if (angleDelta.y() != 0) { // 垂直滚动
		int delta = angleDelta.y() > 0 ? 1 : -1; // 简化为1或-1步长

		// 获取当前中心点
		QPoint center = currentRoi.center();

		// 计算新尺寸（保持中心点不变）
		int newWidth = qBound(minSize, currentRoi.width() + delta, maxSize);
		int newHeight = qBound(minSize, currentRoi.height() + delta, maxSize);

		// 直接设置尺寸（自动调整右下角，保持左上角不变）
		currentRoi.setSize(QSize(newWidth, newHeight));

		// 重新设置中心点（关键优化：一步到位）
		currentRoi.moveCenter(center);

		update();
	}
}
