#include "RobotApp.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RobotApp w;
    w.show();
    return a.exec();
}
