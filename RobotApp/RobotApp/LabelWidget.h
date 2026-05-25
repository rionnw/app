#pragma once

#include <QWidget>

#include <vector>

#include <opencv2/opencv.hpp>

class LabelWidget  : public QWidget
{
	Q_OBJECT

public:
	LabelWidget(QWidget *parent);
	~LabelWidget() = default;

	void setImage(QImage& img);
	void setRois(const std::vector<cv::Rect>& cvRois);
	std::vector<cv::Rect> getRois() const;

public slots:
	void removeLastRoi();
	void resetRois();

private:
	QImage image;

	QVector<QRect> rois;
	QRect currentRoi;

	const QVector<QColor> roiColors{ Qt::red, Qt::green, Qt::blue };

	void paintEvent(QPaintEvent *ev) override;
	void mousePressEvent(QMouseEvent *ev) override;
	void mouseMoveEvent(QMouseEvent *ev) override;
	void wheelEvent(QWheelEvent* ev) override;
};
