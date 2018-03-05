#ifndef QTVIEWER_H
#define QTVIEWER_H

#include <qt4/QtGui/QApplication>
#include <qt4/QtGui/QMouseEvent>
#include <qt4/QtGui/QColor>
#include <qt4/QtCore/QPoint>
#include <qt4/QtOpenGL/QtOpenGL>
#include <qt4/QtGui/QWidget>
#include <GL/glu.h>

class QtViewer : public QGLWidget
{
//    Q_OBJECT
public:
    QtViewer(QWidget *parent = 0);
    ~QtViewer();
protected:
    void initializeGL();
    void resizeGL(int width, int height);
    void paintGL();
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
private:
    void draw();
    int faceAtPosition(const QPoint &pos);
    GLfloat rotationX;
    GLfloat rotationY;
    GLfloat rotationZ;
    QColor faceColors[4];
    QPoint lastPos;
};
#endif
