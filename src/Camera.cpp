/*
 * camera.cpp
 *
 *  Created on: 19-Nov-2011
 *      Author: prateek
 */

#include "Camera.h"


  Camera::Camera(): theta(0), y(0), dTheta(0.04), dy(0.2) {}
  double Camera::getX() {return 10 * cos(theta);}
  double Camera::getY() {return y;}
  double Camera::getZ() {return 10 * sin(theta);}
  void Camera::moveRight() {theta += dTheta;}
  void Camera::moveLeft() {theta -= dTheta;}
  void Camera::moveUp() {y += dy;}
  void Camera::moveDown() {if (y > dy) y -= dy;}
