#ifndef WINDOW1_H
#define WINDOW1_H

#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <QPlainTextEdit>
#include <QMutex>

class window1 : public QMainWindow{
    Q_OBJECT
public:
    window1(QWidget *parent = nullptr);
    void paintEvent(QPaintEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void timerEvent(QTimerEvent *event);

private:
    QImage* m_img;

signals:

};

#endif // WINDOW1_H
