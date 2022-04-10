#include "drawmap.h"

#include <QtGui>
#include <QtDebug>
#include <QThread>
#include <QtWidgets>
#include <iostream>

#include <QMouseEvent>
#include <QKeyEvent>

using namespace std;


DrawMap::DrawMap(QWidget *parent) : QMainWindow(parent){
    // create background image
    mapWidth = 1200;
    mapHeight = 1200;
    m_img = new QImage(mapWidth, mapHeight, QImage::Format_RGB888);
    m_img->fill(QColor(0xff, 0xff, 0xff));

    // set window properties
    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);

    setGeometry(20, 35, mapWidth + 20, mapHeight + 20);

    // set focus to self
    setFocus();
}

void DrawMap::keyPressEvent(QKeyEvent *event){
    QPainter painter;

    // `o` draw grid
    if(event->key() == Qt::Key_O){
        // draw background grid
        cout << "painting grid..." << endl;
        painter.begin(m_img);
        painter.setRenderHint(QPainter::Antialiasing, true);

        int offset;
        int grid_size, nx, ny;
        double x1, x2, y1, y2;

        nx = 100;            // number of grids in x axis
        ny = 100;            // number of grids in y axis
        grid_size = 10;     // pixels of grid size
        offset = 0;         // margin top and left

        // draw vertical lines
        y1 = offset + 0;
        y2 = offset + ny * grid_size;
        for (int i = 0; i <= nx; i++ ){
            x1 = offset + i * grid_size;
            painter.drawLine(x1, y1, x1, y2);
        }

        // draw horizontal lines
        x1 = offset + 0;
        x2 = offset + nx * grid_size;
        for (int i = 0; i <=ny; i++){
            y1 = offset + i * grid_size;
            painter.drawLine(x1, y1, x2, y1);
        }

        painter.end();
    }

    // `Q` quit the programme
    if (event->key() == Qt::Key_Q){
        cout << "quit successfullly" << endl;
        this->close();
    }

    this->update();
}

void DrawMap::paintEvent(QPaintEvent *){
    QPainter painter(this);

    // draw offline image to screen
    painter.drawImage(QPoint(0, 0), *m_img);
}

void DrawMap::mousePressEvent(QMouseEvent *event){
    // set Start point, End point
    // https://blog.csdn.net/owen7500/article/details/51035078
    // https://gitee.com/pi-lab/code_cook/blob/master/gui/qt/2_draw_map/map_viewer.cpp

    if (event->button() == Qt::LeftButton){
        cout << "left button is clicked" << endl;
    }

}

void DrawMap::mouseMoveEvent(QMouseEvent *event){
    // track mouse move to draw obstacles

}

