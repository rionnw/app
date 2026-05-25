#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_RobotApp.h"

#include <QTime>
#include <QTimer>
#include <QSerialPort>

#include <vector>
#include <atomic>

#include <opencv2/opencv.hpp>

#include "ColorCluster.h"
#include "StepOptimizer.h"
#include "LabelWindow.h"

using std::vector;

// 工具栏
enum menuAction
{
	IMG_SAVE,
	IMG_READ,
	IMG_MARK,
	ACTION_TEST
};

const char FRAME_HEADER_1 = 0xAA;
const char FRAME_HEADER_2 = 0x55;
const char FRAME_TAIL = 0xFF;
const char CMD_MOTION = 0x01;
const char CMD_PARAM_WRITE = 0x02;
const char CMD_PARAM_READ = 0x03;
const quint8 MAX_MOTION_LENGTH = 120;

QT_BEGIN_NAMESPACE
namespace Ui { class RobotAppClass; };
QT_END_NAMESPACE

class RobotApp : public QMainWindow
{
	Q_OBJECT

public:
	RobotApp(QWidget* parent = nullptr);
	~RobotApp();

private:
	Ui::RobotAppClass* ui;

	QTimer* camTimer;   // 获取相机帧定时器
	std::array<int, 4>    camOrders; // 相机顺序
	std::array<cv::VideoCapture, 4> cameras; // 相机对象
	cv::Mat          cubeImg;
	bool             isCamOpened = false; // 相机是否打开

	vector<cv::Rect> rois; // ROI 区域
	// CtrlParams params;

	QTimer* lcdTimer;    // 更新 LCD 计时定时器
	QTime   startTime;   // LCD 秒表基准时间

	ColorCluster* cluster;
	StepOptimizer* optimizer;
	LabelWindow* labelWindow;

	QSerialPort* serialPort;
	QStringList  serialPortsInfo;
	std::atomic<bool> isSerialPortOpen = false; // 串口是否打开

	void getCurFrame();
	void updateImgShow();
	bool eventFilter(QObject* obj, QEvent* event);   // 添加事件过滤器声明
	QStringList getSerialPortsInfo();

	void sendParamWriteCommand(int left, int right, double motorSpeed, double gripperSpeed);

	void sendParamReadCommand();

	int sendData(const QByteArray& data);

	int sendMsg(const QByteArray& sendBuf);

private slots:
	void menuActionTriggered(int op);

	void openCamBtnClicked();
	void runBtnClicked();
	void labelBtnClicked();

	void updateLCD();

	void openSerialPort();
	void refreshSerialPort();
	void recvMsg();

	void writeParamBtnClicked();
	void readParamBtnClicked();

	void handleLabelWindowVisibility(bool visible);
};