#pragma once

#include <QMainWindow>
#include "ui_LabelWindow.h"

QT_BEGIN_NAMESPACE
namespace Ui { class LabelWindowClass; };
QT_END_NAMESPACE

class LabelWindow : public QMainWindow
{
	Q_OBJECT

public:
	LabelWindow(QWidget *parent = nullptr);
	~LabelWindow();

	void setLabelImage(QImage img);

	void setLabelRois(const std::vector<cv::Rect>& rois);

	std::vector<cv::Rect> getLabelRois() const;

signals:
	void visibilityChanged(bool visible);

protected:
	void showEvent(QShowEvent* event) override;
	void hideEvent(QHideEvent* event) override;

private:
	Ui::LabelWindowClass *ui;
};
