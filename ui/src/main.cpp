#include "../include/mainwindow.h"
#include <QApplication>
#include "../include/temp.h"
int main(int argc, char *argv[])
{
temp::test();
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
