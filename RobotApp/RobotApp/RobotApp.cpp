#include "RobotApp.h"

#include <QSerialPortInfo>
#include <QMessageBox>

#include "ColorCluster.h"
#include "FileUtils.hpp"
#include "AuthUtils.hpp"
#include "ImgUtils.hpp"

#include <QDebug>

RobotApp::RobotApp(QWidget* parent)
	: QMainWindow(parent)
	, ui(new Ui::RobotAppClass())
{
	ui->setupUi(this);

	// 时间授权
	{
		// 获取当前日期
		QDate currentDate = QDate::currentDate();
		// 比较日期是否在授权范围内
		if (currentDate > QDate(2025, 7, 30)) {
			exit(-1);  // 如果超过授权日期，退出程序
		}
	}

	// 工具栏
	{
		connect(ui->imgSaveAction, &QAction::triggered, this, [this] {menuActionTriggered(IMG_SAVE); });
		connect(ui->imgReadAction, &QAction::triggered, this, [this] {menuActionTriggered(IMG_READ); });
		connect(ui->testAction, &QAction::triggered, this, [this] {menuActionTriggered(ACTION_TEST); });
	}

	// 魔方图像
	{
		// 初始化图像
		cubeImg = cv::Mat::zeros(960, 1280, CV_8UC3);

		// 初始化相机顺序
		std::iota(camOrders.begin(), camOrders.end(), 0);

		// 安装事件过滤器，为QLabel提供鼠标点击事件处理
		ui->curImgLabel->installEventFilter(this);

		// 按键槽函数
		connect(ui->openCamBtn, &QPushButton::clicked, this, &RobotApp::openCamBtnClicked);
		connect(ui->runBtn, &QPushButton::clicked, this, &RobotApp::runBtnClicked);
		connect(ui->labelImgBtn, &QPushButton::clicked, this, &RobotApp::labelBtnClicked);

		// 初始化采图定时器并连接槽函数
		camTimer = new QTimer(this);
		camTimer->setInterval(33);
		connect(camTimer, &QTimer::timeout, this, &RobotApp::updateImgShow);
	}

	rois = initRois();
	// params = initParams();
	// drawCube();
	cluster = new ColorCluster;
	optimizer = new StepOptimizer;
	labelWindow = new LabelWindow(this);
	labelWindow->setWindowModality(Qt::ApplicationModal);

	connect(labelWindow, &LabelWindow::visibilityChanged, this, &RobotApp::handleLabelWindowVisibility);

	// 计时器
	{
		// 初始化计时定时器并连接槽函数
		lcdTimer = new QTimer(this);
		// 将字符串最后一个0去掉
		ui->lcdNumber->display("00:00.00");
		connect(lcdTimer, &QTimer::timeout, this, &RobotApp::updateLCD);
	}

	// 串口
	{
		serialPort = new QSerialPort();
		serialPortsInfo = getSerialPortsInfo();

		ui->serialInfoBox->clear();
		ui->serialInfoBox->addItems(serialPortsInfo);

		if (!serialPortsInfo.empty()) {
			serialPort->setPortName(serialPortsInfo.at(0));
			openSerialPort();
		}

		connect(serialPort, &QSerialPort::readyRead, this, &RobotApp::recvMsg);
		connect(ui->serialOpenBtn, &QPushButton::clicked, this, &RobotApp::openSerialPort);
		connect(ui->serialRefreshBtn, &QPushButton::clicked, this, &RobotApp::refreshSerialPort);
	}

	// 控制参数
	{
		connect(ui->writeParamBtn, &QPushButton::clicked, this, &RobotApp::writeParamBtnClicked);
		connect(ui->readParamBtn, &QPushButton::clicked, this, &RobotApp::readParamBtnClicked);
	}

	// 信息输出
	{
		ui->msgOutText->setReadOnly(true);
	}

	// 测试
	//{
	//	std::ifstream file("bench.cubes");

	//	std::vector<std::string> lines;
	//	std::string line;

	//	int all = 0;
	//	int cnt = 0;
	//	// 使用std::getline逐行读取，处理各种换行符
	//	while (std::getline(file, line)) {
	//		cnt++;
	//		auto steps = optimizer->getRobotSteps(line);
	//		spdlog::info("Cube: {}, Steps: {}", line, steps.size());
	//		all += steps.size();
	//	}

	//	file.close();
	//	spdlog::info("Total Cubes: {}, Average Steps: {}", cnt, all / cnt);
	//}
}

/*
 * @brief 开/关相机按钮 槽函数
 */
void RobotApp::openCamBtnClicked() 
{

	camTimer->stop();

	ui->openCamBtn->setEnabled(false);

	if (!isCamOpened) {
		ui->msgOutText->appendPlainText("Cameras Opening...");
		for (int i = 0; i < 4; ++i) {
			cameras[i].open(i, cv::CAP_DSHOW);
			if (cameras[i].isOpened()) {
				cameras[i].set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));   // 视频流格式
				cameras[i].set(cv::CAP_PROP_FRAME_WIDTH, 640);
				cameras[i].set(cv::CAP_PROP_FRAME_HEIGHT, 480);
			}
			else {
				ui->msgOutText->appendPlainText(QString("%1 %2").arg("Failed to Open Capture.").arg(i));
			}
		}
		isCamOpened = true;
		ui->openCamBtn->setText(QStringLiteral("关闭相机"));
		ui->msgOutText->appendPlainText("Opened Cameras.");
	}
	else {
		ui->msgOutText->appendPlainText("Cameras Closing...");
		for (int i = 0; i < 4; ++i) {
			if (cameras[i].isOpened()) {
				cameras[i].release();
			}
		}
		isCamOpened = false;
		ui->openCamBtn->setText(QStringLiteral("打开相机"));
		ui->msgOutText->appendPlainText("Cameras Closed.");
	}

	ui->openCamBtn->setEnabled(true);

	camTimer->start(33);
}

void RobotApp::runBtnClicked() 
{
	auto tick = std::chrono::high_resolution_clock::now();
	camTimer->stop();
	lcdTimer->stop();
	ui->lcdNumber->display("00:00.00");
	startTime = QTime::currentTime();

	if (!isCamOpened) {
		openCamBtnClicked();
	}

	// 1.采集最新帧图像 进行颜色聚类
	getCurFrame();

	// 2.进行颜色聚类
	auto res = cluster->cluster(cubeImg, rois);
	ui->msgOutText->appendPlainText(QString::fromStdString(res.msg));

	// 3.魔方可解 进行步骤转换
	if (res.isCubeSolved) {
		auto steps = optimizer->getRobotSteps(res.cubeFace);
		if (!steps.empty()) {
			// 4.发送串口
			if (isSerialPortOpen) {
				QByteArray sendBuf = QByteArray::fromStdString(steps);
				if (!sendBuf.isEmpty() && sendMsg(sendBuf) == sendBuf.size()) {
					lcdTimer->start(50);
				}
			}
			else {
				ui->msgOutText->appendPlainText("Serial Port not Open !");
			}
		}
	}

	spdlog::info("cluster cost time: {} ms",
		std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - tick)
		.count());


	camTimer->start(33);
}


int RobotApp::sendMsg(const QByteArray& sendBuf)
{
	if (!serialPort->isOpen()) {
		QMessageBox::information(NULL,
			QString::fromLocal8Bit("串口连接"),
			QString::fromLocal8Bit("串口未打开，请检查！"));
		return 0;
	}

	serialPort->clear();
	// 帧头
	char temp = 0XAA;
	int  len = serialPort->write(&temp, 1);
	// 帧数据
	if (len == 1) {
		len += serialPort->write(sendBuf);
	}
	for (; len < 149; ++len) {
		temp = 'Z';
		serialPort->write(&temp, 1);
	}
	// 帧尾
	if (len == 149) {
		temp = 0XBB;
		len += serialPort->write(&temp, 1);
	}
	if (len == 150)
		return sendBuf.size();
	else
		return 0;
}

void RobotApp::labelBtnClicked()
{
	// 打开标定窗口
	if (labelWindow->isHidden()) {
		getCurFrame();
		labelWindow->setLabelRois(rois); // 设置标定区域
		labelWindow->setLabelImage(cvMatToQImage(cubeImg));
		labelWindow->show();
	}
	else {
		labelWindow->hide();
	}
}

/**
 * 读取当前帧图像
 * - 从摄像头或图片文件获取图像数据
 * - 处理多摄像头拼接
 */
void RobotApp::getCurFrame() 
{
	// 初始化输出图像（1280x960，4个摄像头拼接）
	cubeImg = cv::Mat::zeros(960, 1280, CV_8UC3);

	// 定义每个摄像头在输出图像中的位置
	const std::array<cv::Rect, 4> regions = {
		cv::Rect(0, 0, 640, 480),    // 左上
		cv::Rect(640, 0, 640, 480),  // 右上
		cv::Rect(0, 480, 640, 480),  // 左下
		cv::Rect(640, 480, 640, 480) // 右下
	};

	bool allCamerasDisconnected = true;

	for (int i = 0; i < 4; ++i) {
		const int camIndex = camOrders[i];

		// 检查相机是否断开连接
		if (cameras[camIndex].isOpened()) {
			cv::Mat frame;
			bool grabSuccess = cameras[camIndex].grab();

			if (grabSuccess && cameras[camIndex].retrieve(frame)) {
				allCamerasDisconnected = false;

				// 调整大小并直接写入结果图像
				cv::Mat targetROI = cubeImg(regions[i]);
				cv::resize(frame, targetROI, cv::Size(640, 480));
			}
			else {
				// 处理相机断开连接的情况
				ui->msgOutText->appendPlainText("Camera " + QString::number(camIndex) + " disconnected!");
				cameras[camIndex].release();
				cubeImg(regions[i]).setTo(cv::Scalar(100, 100, 100));
			}
		}
		else {
			// 相机未打开，使用黑色图像
			cubeImg(regions[i]).setTo(cv::Scalar(100, 100, 100));
		}
	}

	// 如果所有相机都断开连接，停止定时器
	if (allCamerasDisconnected && isCamOpened) {
		ui->msgOutText->appendPlainText("All cameras disconnected! Stopping capture.");
		isCamOpened = false;
		ui->openCamBtn->setText(QStringLiteral("打开相机"));
		camTimer->stop();
	}

	// cubeImg = cv::imread("img.png");
}

/**
 * 获取图片后 绘制 ROI 并显示到界面
 */
void RobotApp::updateImgShow()
{
	// 获取当前帧
	getCurFrame();

	for (int i = 0; i < rois.size(); i += 3) {
		cv::rectangle(cubeImg, rois[i + 0], cv::Scalar(0, 0, 255), 2, cv::LINE_8, 0);
		cv::rectangle(cubeImg, rois[i + 1], cv::Scalar(0, 255, 0), 2, cv::LINE_8, 0);
		cv::rectangle(cubeImg, rois[i + 2], cv::Scalar(255, 0, 0), 2, cv::LINE_8, 0);
	}

	QImage curImg = cvMatToQImage(cubeImg);
	// 图片缩放：W H 为 宽 高 1280 960 缩为 960 720
	static const QSize targetSize(960, 720);
	curImg = curImg.scaled(targetSize, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

	ui->curImgLabel->setPixmap(QPixmap::fromImage(curImg));
}

/**
 * 更新 LCD计时器 显示
 */
void RobotApp::updateLCD() 
{
	// 计算当前时间与开始时间的差值（毫秒）
	const int elapsedMSecs = startTime.msecsTo(QTime::currentTime());

	// 创建临时时间对象并显示
	QTime displayTime(0, 0, 0, 0);
	displayTime = displayTime.addMSecs(elapsedMSecs);

	// 格式化时间显示为 "mm:ss.zz"（移除最后一位毫秒）
	const QString timeStr = displayTime.toString("mm:ss.zzz");
	ui->lcdNumber->display(timeStr.left(timeStr.length() - 1));
}

bool RobotApp::eventFilter(QObject* obj, QEvent* event)
{
	// 静态变量用于跟踪点击次数和最后点击的图像ID
	static int clickTimes = 0;
	static int lastImgId = 0;

	// 检查事件是否为指定QLabel的鼠标左键点击
	if (obj == ui->curImgLabel && event->type() == QEvent::MouseButtonPress) {
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);

		if (mouseEvent->button() == Qt::LeftButton) {
			// 计算当前点击的图像ID（2x2网格布局）
			const int curImgId = mouseEvent->x() / 480 + (mouseEvent->y() / 360) * 2;

			// 检查是否满足交换条件：两次点击不同区域
			if (++clickTimes > 1 && lastImgId != curImgId) {
				// 重置计数器
				clickTimes = 0;

				// 执行摄像头顺序交换
				std::swap(camOrders[lastImgId], camOrders[curImgId]);
			}

			// 更新最后点击的图像ID
			lastImgId = curImgId;

			// 标记事件已处理
			return true;
		}
	}

	// 默认处理方式
	return QWidget::eventFilter(obj, event);
}

/**
 * 开/关串口
 */
void RobotApp::openSerialPort()
{
	// 判断串口开启状态
	if (serialPort->isOpen()) {
		// 若串口已经打开，则关闭它，设置指示灯为红色，设置按钮显示“打开串口”
		serialPort->clear();
		serialPort->close();
		ui->serialOpenBtn->setText(QStringLiteral("打开"));
		ui->serialStatusLabel->setText(QStringLiteral("已关闭"));
		isSerialPortOpen = false;
	}
	else {
		// 若串口没有打开，则打开选择的串口，设置指示灯为绿色，设置按钮显示“关闭串口”
		serialPort->setPortName(ui->serialInfoBox->currentText());
		if (serialPort->open(QIODevice::ReadWrite)) {
			serialPort->setBaudRate(QSerialPort::Baud115200);
			serialPort->setDataBits(QSerialPort::Data8);
			serialPort->setParity(QSerialPort::NoParity);
			serialPort->setStopBits(QSerialPort::OneStop);
			serialPort->setFlowControl(QSerialPort::NoFlowControl);
			ui->serialOpenBtn->setText(QStringLiteral("关闭"));
			ui->serialStatusLabel->setText(QStringLiteral("已连接"));
			isSerialPortOpen = true;
		}
	}
}

/**
 * 刷新串口列表并更新UI显示
 * - 检测串口设备变化
 * - 保持已打开串口的选中状态
 * - 在串口断开时尝试重新连接
 */
void RobotApp::refreshSerialPort()
{
	// 获取新旧串口信息列表
	const QStringList oldPorts = serialPortsInfo;
	serialPortsInfo = getSerialPortsInfo();

	// 保存当前选中的串口
	const QString currentPort = ui->serialInfoBox->currentText();

	// 检查串口列表是否发生变化
	// 如果新旧串口列表大小不同或内容不同，则更新UI
	auto oldPortsSet = QSet<QString>(oldPorts.begin(), oldPorts.end());
	auto serialPortsSet = QSet<QString>(serialPortsInfo.begin(), serialPortsInfo.end());
	if (oldPorts.size() != serialPortsInfo.size() || !oldPortsSet.contains(serialPortsSet)) {

		// 清空并重新填充串口选择框
		ui->serialInfoBox->clear();

		if (serialPort->isOpen()) {
			// 优先添加已打开的串口（保持选中状态）
			ui->serialInfoBox->addItem(serialPort->portName());

			// 添加其他可用串口（排除已打开的）
			for (const QString& port : serialPortsInfo) {
				if (port != serialPort->portName()) {
					ui->serialInfoBox->addItem(port);
				}
			}
		}
		else {
			// 无打开串口时，直接添加所有可用串口
			ui->serialInfoBox->addItems(serialPortsInfo);
		}

		// 处理已断开的串口
		if (!serialPortsInfo.contains(currentPort) && serialPort->isOpen()) {
			openSerialPort(); // 关闭当前串口

			// 从列表中移除无效串口
			const int currentIdx = ui->serialInfoBox->findText(currentPort);
			if (currentIdx >= 0) {
				ui->serialInfoBox->removeItem(currentIdx);
			}
		}
	}
}

/**
 * 获取串口列表
 */
QStringList RobotApp::getSerialPortsInfo()
{
	QStringList serialPortInfo;

	// 获取可用的串口列表
	for (auto& port : QSerialPortInfo::availablePorts()) {
		serialPortInfo << port.portName();
	}
	return serialPortInfo;
}

void RobotApp::sendParamWriteCommand(int left, int right, double motorSpeed, double gripperSpeed)
{
	QByteArray data;
	data.append(FRAME_HEADER_1);
	data.append(FRAME_HEADER_1);
	data.append(FRAME_HEADER_2);
	data.append(CMD_PARAM_WRITE);
	data.append(24); // 四个int和两个double的长度
	data.append(reinterpret_cast<const char*>(&left), 4);
	data.append(reinterpret_cast<const char*>(&right), 4);
	data.append(reinterpret_cast<const char*>(&motorSpeed), 8);
	data.append(reinterpret_cast<const char*>(&gripperSpeed), 8);

	for (int i = 5 + 24; i < 150 - 1; ++i) {
		data.append(0xFF);
	}

	data.append(FRAME_TAIL);
	sendData(data);
}

void RobotApp::sendParamReadCommand()
{
	QByteArray data;
	data.append(FRAME_HEADER_1);
	data.append(FRAME_HEADER_2);
	data.append(CMD_PARAM_READ);

	for (int i = 3; i < 150 - 1; ++i) {
		data.append('Z');
	}

	data.append(FRAME_TAIL);
	sendData(data);
}

/**
 * 串口发送函数 返回发送字节数
 */
 /**
  * 向串口发送格式化数据帧
  * 帧格式: 帧头(0xAA) + 数据 + 填充(补全至149字节) + 帧尾(0xBB)
  * @param sendBuf 待发送的数据
  * @return 成功发送的数据长度，失败返回0
  */
int RobotApp::sendData(const QByteArray& data)
{
	// 检查串口状态
	if (!serialPort->isOpen()) {
		QMessageBox::information(nullptr, QStringLiteral("串口连接"), QStringLiteral("串口未打开，请检查！"));
		return 0;
	}

	qint64 bytesWritten = serialPort->write(data);

	// 检查发送结果
	if (bytesWritten == data.size()) {
		spdlog::info("sendData size: {}", bytesWritten);
		serialPort->waitForBytesWritten();
		return data.size();  // 返回原始数据长度
	}
	else {
		spdlog::warn("Failed to send full frame: {} bytes sent.", bytesWritten);
		return 0;
	}
}

/**
 * 串口接收槽函数 判断结束标志
 */
void RobotApp::recvMsg()
{
	QByteArray info = serialPort->readAll();
	qDebug() << info;
	if (info.contains("ND")) {
		serialPort->clear();
		lcdTimer->stop();
		updateLCD();
	}
	else if (info.contains("SG")) {
		serialPort->clear();
	}
	else if (info.contains("OK")) {
		// 处理成功响应
		ui->msgOutText->appendPlainText(QStringLiteral("参数写入成功！"));
	}
	else if (info.contains("ER")) {
		// 处理错误响应
		ui->msgOutText->appendPlainText(QStringLiteral("参数写入失败！请检查设备连接。"));
	}
	else {
		// 处理其他信息
		ui->msgOutText->appendPlainText(QString::fromUtf8(info));
	}
}

void RobotApp::writeParamBtnClicked()
{
	// 写入参数逻辑
	double airParam = ui->airParamBox->value();
	double motorParam = ui->motorParamBox->value();

	int leftParam = ui->leftBox->value();
	int rightParam = ui->rightBox->value();

	spdlog::info("Writing parameters: AirParam: {}, MotorParam: {}, LeftParam: {}, RightParam: {}",
		airParam, motorParam, leftParam, rightParam);

	// 发送参数到串口
	sendParamWriteCommand(leftParam, rightParam, airParam, motorParam);
}

void RobotApp::readParamBtnClicked()
{
	// 读取参数逻辑
	// 这里可以添加读取参数的代码
	sendParamReadCommand();
}

void RobotApp::handleLabelWindowVisibility(bool visible)
{
	if (visible) {
		camTimer->stop(); // 子窗口显示时停止定时器
	}
	else {
		rois = labelWindow->getLabelRois(); // 获取标定后的ROI
		saveRois(rois);
		camTimer->start(33); // 子窗口隐藏时启动定时器
	}
}

void RobotApp::menuActionTriggered(int op) {
	switch (op) {
	case IMG_SAVE:
		// 保存当前图像
		break;
	case IMG_READ:
		// 读取图像
		break;
	case ACTION_TEST:
		// 测试功能
		break;
	default:
		break;
	}
}

RobotApp::~RobotApp()
{
	delete cluster;
	delete optimizer;

	// 串口相关
	if (serialPort->isOpen()) {
		serialPort->close();
	}
	delete serialPort;

	delete ui;
}
