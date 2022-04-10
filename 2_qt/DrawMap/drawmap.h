#ifndef DRAWMAP_H
#define DRAWMAP_H

#include <QMainWindow>
#include <QObject>
#include <QWidget>

class DrawMap : public QMainWindow{
    Q_OBJECT
public:
    explicit DrawMap(QWidget *parent = nullptr);
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent * /* event */);


private:
    QImage * m_img, dst;
    int mapWidth;
    int mapHeight;


signals:

};

#endif // DRAWMAP_H
