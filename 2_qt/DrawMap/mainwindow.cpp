#include <stdio.h>
#include <stdlib.h>

#include <QThread>
#include <QtGui>
#include <QDebug>

#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // create background image
        m_canvasW = 800;
        m_canvasH = 800;
        m_img = new QImage(m_canvasW, m_canvasH, QImage::Format_RGB888);
        m_img->fill(QColor(0xff, 0xff, 0xff));

        // set window properties
        setBackgroundRole(QPalette::Base);
        setAutoFillBackground(true);

        setGeometry(20, 35, m_canvasW+20, m_canvasH+20);

        // set focus to self
        setFocus();
}

void MainWindow::timerEvent(QTimerEvent *event)
{

}


void MainWindow::keyPressEvent(QKeyEvent *event)
{
    QPainter    painter;

    // 'o' - draw grid
    if( event->key() == Qt::Key_O ) {
        // begin draw to background image
        painter.begin(m_img);
        painter.setRenderHint(QPainter::Antialiasing, true);

        int         offset;
        int         grid_size, nx, ny;
        double      x1, y1, x2, y2;

        offset = 10;
        grid_size = 15;
        nx = 40;
        ny = 40;

        // draw vertical lines
        y1 = offset+0;
        y2 = offset+ny*grid_size;
        for(int i=0; i<=nx; i++) {
            x1 = offset+i*grid_size;
            painter.drawLine(x1, y1, x1, y2);
        }

        // draw horiztonal lines
        x1 = offset+0;
        x2 = offset+nx*grid_size;
        for(int i=0; i<=ny; i++) {
            y1 = offset+i*grid_size;
            painter.drawLine(x1, y1, x2, y1);
        }

        painter.end();
    }

    // 'c' - clean canvas
    if( event->key() == Qt::Key_C ) {
        m_img->fill(QColor(0xff, 0xff, 0xff));
    }

    // 'q' - quit the program
    if( event->key() == Qt::Key_Q ) {
        this->close();
    }

    this->update();
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
}

void MainWindow::paintEvent(QPaintEvent * /* event */)
{
    QPainter    painter(this);

    // draw offline image to screen
    painter.drawImage(QPoint(0, 0), *m_img);
}

MainWindow::~MainWindow()
{
    delete ui;
}

