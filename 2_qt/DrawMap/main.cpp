#include "mainwindow.h"
#include "window1.h"
#include "drawmap.h"

#include <QApplication>
#include <QtGui>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // create main window
    MainWindow w;
    w.setWindowTitle("simple title");
    w.show();

    window1 myWindow;
    myWindow.setWindowTitle("window 1");
    myWindow.show();

    DrawMap mapWindow;
    mapWindow.setWindowTitle("Map");
    mapWindow.show();

    // Qt GUI loop
    a.exec();

    return 0;
}
