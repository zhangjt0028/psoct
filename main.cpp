#include "mainwidget.h"
#include <QApplication>
#include <QFile>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    mainWidget w;
    w.setStyleSheet("background-color:black;");

    w.show();

    QFile styleFile(":\\sty\\style.qss");
        if(styleFile.open(QIODevice::ReadOnly))
        {
            qDebug("open success");
            QString setStyleSheet(styleFile.readAll());
            a.setStyleSheet(setStyleSheet);
            styleFile.close();
        }
        else
        {
            qDebug("Open failed");
        }

    return a.exec();
}
