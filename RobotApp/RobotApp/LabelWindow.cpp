#include "LabelWindow.h"

LabelWindow::LabelWindow(QWidget* parent)
	: QMainWindow(parent)
	, ui(new Ui::LabelWindowClass())
{
	ui->setupUi(this);

	connect(ui->resetRoiBtn, &QPushButton::clicked, ui->labelWidget, &LabelWidget::resetRois);
	connect(ui->removeLastRoiBtn, &QPushButton::clicked, ui->labelWidget, &LabelWidget::removeLastRoi);
}

void LabelWindow::setLabelImage(QImage img) 
{
	ui->labelWidget->setImage(img);
};

void LabelWindow::setLabelRois(const std::vector<cv::Rect>& rois) 
{
	ui->labelWidget->setRois(rois);
};

std::vector<cv::Rect> LabelWindow::getLabelRois() const 
{
	return ui->labelWidget->getRois();
};

void LabelWindow::showEvent(QShowEvent* event)
{
	emit visibilityChanged(true);
	QWidget::showEvent(event);
}

void LabelWindow::hideEvent(QHideEvent* event)
{
	emit visibilityChanged(false);
	QWidget::hideEvent(event);
}

LabelWindow::~LabelWindow()
{
	delete ui;
}
