#pragma once

#include <QCryptographicHash>
#include <QDate>
#include <QFile>
#include <QString>
#include <QTextStream>

#include <QAESEncryption.h>

// AES加密算法的 key
const QString key = "aowang's copyright";

/**
 * 获取CPU ID (前16个字符的十六进制字符串)
 * 使用CPUID指令获取处理器信息
 * @return 包含CPU ID的QString
 */
QString getCpuId()
{
    // 存储CPUID返回的4个32位值
    unsigned int cpuidInfo[4] = { 0 };

    // 调用CPUID指令获取处理器信息(叶函数0x01)
    __cpuid(reinterpret_cast<int*>(cpuidInfo), 1);

    // 直接构建格式化的16进制字符串，确保每个部分都是8个字符宽
    return QString("%1%2")
        .arg(cpuidInfo[3], 8, 16, QChar('0')).toUpper()  // EDX寄存器值
        .arg(cpuidInfo[0], 8, 16, QChar('0')).toUpper(); // EAX寄存器值
}

/**
 * 检查授权信息
 */
bool isAuthorization()
{
    return true;
    auto encryption =
        new QAESEncryption(QAESEncryption::AES_128, QAESEncryption::ECB, QAESEncryption::ZERO);

    auto hardwareInfo = getCpuId();
    auto nowDate = QDateTime::currentDateTime().date();

    // 读取授权文件
    QFile readFile;
    readFile.setFileName("./key.lic");
    QString data;
    if (readFile.open(QIODevice::ReadOnly)) {
        QTextStream in(&readFile);
        data = in.readLine();
    }
    readFile.close();

    // 判断是否授权
    QByteArray  hashKey = QCryptographicHash::hash(key.toUtf8(), QCryptographicHash::Md5);
    QByteArray  decodedText = encryption->decode(QByteArray::fromBase64(data.toLatin1()), hashKey);
    QStringList infos = QString::fromLatin1(decodedText).split('$');

    if (infos.at(0) == hardwareInfo && QDate::fromString(infos.at(1), "yyyy-MM-dd") >= nowDate) {
        return true;
    }

    // 如果没有授权，就把硬件信息和当前时间写入文件
    QFile writeFile;
    writeFile.setFileName("./key.lic");
    if (writeFile.open(QIODevice::WriteOnly)) {
        QTextStream out(&writeFile);
        out << getCpuId() << "$" << nowDate.toString("yyyy-MM-dd");
    }
    writeFile.close();
    return false;
}
