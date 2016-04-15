/****************************************************************************
**
** Copyright (C) 2013 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
**     of its contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtWidgets>
#include <QtOpenGL>
#include <QThread>

#include "glwidget.h"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

using namespace std;
using namespace Eigen;


GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    logo = 0;
    xRot = 0;
    yRot = 0;
    zRot = 0;
    zoom = 4;

    niteration = 0;

    Vector4d goal_center;
    goal_center << 0,0,1,1;
    double goal_ray = 0.1;
    needlerrt.setGoalArea(goal_center,goal_ray);

    Vector4d center_o1;
    center_o1 << 0.0,0.0,0.5,1.0;
    double size_o1 = 0.05;
    Obstacle o1(center_o1,size_o1);
    needlerrt.insertObstacle(o1);

    Vector4d center_o2;
    center_o2 << 0.0,0.5,0.5,1.0;
    double size_o2 = 0.1;
    Obstacle o2(center_o2,size_o2);
    needlerrt.insertObstacle(o2);

    qtGreen = QColor::fromCmykF(0.40, 0.0, 1.0, 0.0);
    qtPurple = QColor::fromCmykF(0.39, 0.39, 0.0, 0.0);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateEdges()));
    timer->start(50);
}
//! [0]

//! [1]
GLWidget::~GLWidget()
{
}
//! [1]

//! [2]
QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}
//! [2]

//! [3]
QSize GLWidget::sizeHint() const
//! [3] //! [4]
{
    return QSize(400, 400);
}
//! [4]

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

//! [5]
void GLWidget::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}
//! [5]

void GLWidget::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
        updateGL();
    }
}

void GLWidget::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
        updateGL();
    }
}

//! [6]
void GLWidget::initializeGL()
{
    qglClearColor(qtPurple.dark());

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
//    glEnable(GL_LIGHTING);
//    glEnable(GL_LIGHT0);
    glEnable(GL_MULTISAMPLE);
//    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
//    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}


void static drawCube(const Vector4d& center, double size){
    glPushMatrix();
        glTranslatef(center.x(),center.y(),center.z());
        glScalef(size,size,size);
        glBegin(GL_QUADS);
//           glColor3f(0.0f,1.0f,0.0f);		  // Set The Color To Green
           glVertex3f( 1.0f, 1.0f,-1.0f);		  // Top Right Of The Quad (Top)
           glVertex3f(-1.0f, 1.0f,-1.0f);		  // Top Left Of The Quad (Top)
           glVertex3f(-1.0f, 1.0f, 1.0f);		  // Bottom Left Of The Quad (Top)
           glVertex3f( 1.0f, 1.0f, 1.0f);		  // Bottom Right Of The Quad (Top)
//           glColor3f(1.0f,0.5f,0.0f);		  // Set The Color To Orange
           glVertex3f( 1.0f,-1.0f, 1.0f);		  // Top Right Of The Quad (Bottom)
           glVertex3f(-1.0f,-1.0f, 1.0f);		  // Top Left Of The Quad (Bottom)
           glVertex3f(-1.0f,-1.0f,-1.0f);		  // Bottom Left Of The Quad (Bottom)
           glVertex3f( 1.0f,-1.0f,-1.0f);		  // Bottom Right Of The Quad (Bottom)
//           glColor3f(1.0f,0.0f,0.0f);		  // Set The Color To Red
           glVertex3f( 1.0f, 1.0f, 1.0f);		  // Top Right Of The Quad (Front)
           glVertex3f(-1.0f, 1.0f, 1.0f);		  // Top Left Of The Quad (Front)
           glVertex3f(-1.0f,-1.0f, 1.0f);		  // Bottom Left Of The Quad (Front)
           glVertex3f( 1.0f,-1.0f, 1.0f);		  // Bottom Right Of The Quad (Front)
//           glColor3f (1.0f,1.0f,0.0f);		  // Set The Color To Yellow
           glVertex3f( 1.0f,-1.0f,-1.0f);		  // Bottom Left Of The Quad (Back)
           glVertex3f(-1.0f,-1.0f,-1.0f);		  // Bottom Right Of The Quad (Back)
           glVertex3f(-1.0f, 1.0f,-1.0f);		  // Top Right Of The Quad (Back)
           glVertex3f( 1.0f, 1.0f,-1.0f);		  // Top Left Of The Quad (Back)
//           glColor3f(0.0f,0.0f,1.0f);		  // Set The Color To Blue
           glVertex3f(-1.0f, 1.0f, 1.0f);		  // Top Right Of The Quad (Left)
           glVertex3f(-1.0f, 1.0f,-1.0f);		  // Top Left Of The Quad (Left)
           glVertex3f(-1.0f,-1.0f,-1.0f);		  // Bottom Left Of The Quad (Left)
           glVertex3f(-1.0f,-1.0f, 1.0f);		  // Bottom Right Of The Quad (Left)
//           glColor3f(1.0f,0.0f,1.0f);		  // Set The Color To Violet
           glVertex3f( 1.0f, 1.0f,-1.0f);		  // Top Right Of The Quad (Right)
           glVertex3f( 1.0f, 1.0f, 1.0f);		  // Top Left Of The Quad (Right)
           glVertex3f( 1.0f,-1.0f, 1.0f);		  // Bottom Left Of The Quad (Right)
           glVertex3f( 1.0f,-1.0f,-1.0f);		  // Bottom Right Of The Quad (Right)

         glEnd();
  glPopMatrix();
}

//! [7]
void GLWidget::paintGL()
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(0.0, -3.0, -10.0);
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
    glScalef(zoom,zoom,zoom);

    vector<Obstacle> obstacles = needlerrt.getObstacles();
    for (uint i=0; i < obstacles.size();i++){
        glColor3f(0.0f,1.0f,0.0f);
        drawCube(obstacles[i].getCenter(), obstacles[i].getSize());
    }
    Vector4d v;
    v << needlerrt.px, needlerrt.py, needlerrt.pz, 1;
    drawCube(v,0.01);

    vector<NVertex*>& verteces = needlerrt.getNeedleTree().getListOfVertex();
    set<NVertex*> colored_verteces;

    if (needlerrt.isFinished()){
        NVertex * current = needlerrt.getGoalVertex();
        colored_verteces.insert(current);
        while(current != NULL){
            current = current->getParent();
            colored_verteces.insert(current);
        }
    }

    for (uint j = 1; j < verteces.size(); j++){
        if (needlerrt.isFinished() && colored_verteces.find(verteces[j]) != colored_verteces.end())
            glColor3f(1.0f,0.0f,0.0f);
        else
            glColor3f(1.0f,1.0f,0.0f);
        glBegin(GL_LINE_STRIP);
        const vector<Vector4d> &points=verteces[j]->getDiscretized();
        for(vector<Vector4d>::const_iterator it = points.begin(); it != points.end(); ++it)
            glVertex3f((*it)[0], (*it)[1], (*it)[2]);
        glEnd();
    }

    // draw the goal area
    glColor3f(1.0f,0.0f,0.0f);
    drawCube(needlerrt.getCenterGoalArea(), needlerrt.getSizeGoalArea());


}
//! [7]

//! [8]
void GLWidget::resizeGL(int width, int height)
{
    glViewport(0,0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
#ifdef QT_OPENGL_ES_1
    glOrthof(-7.12, +7.12, -3.84, +3.84, 4.0, 15.0);
#else
    glOrtho(-7.12, +7.12, -3.84, +3.84, 4.0, 15.0);
#endif
    glMatrixMode(GL_MODELVIEW);

}
//! [8]

//! [9]
void GLWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::MidButton) {
        if (this->timer->isActive())
            this->timer->stop();
        else
            this->timer->start();
    }
    lastPos = event->pos();
}
//! [9]

//! [10]
void GLWidget::wheelEvent(QWheelEvent *event){
  zoom *=(1.0+(event->delta()/120.0)/3.0);
  updateGL();
}
void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();
    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot + 8 * dy);
        setYRotation(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot + 8 * dy);
        setZRotation(zRot + 8 * dx);
    }
    lastPos = event->pos();
}

void GLWidget::updateEdges()
{
    if (! needlerrt.isFinished()){
        for(int i=0;i<ITER_PER_VISUALIZ;i++){
            this->needlerrt.makeStep();
            this->niteration++;
        }
        cout << this->niteration<< endl;
    }

    updateGL();

}

//! [10]
