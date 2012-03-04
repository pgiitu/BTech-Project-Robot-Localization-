/*
 * camera.h
 *
 *  Created on: 19-Nov-2011
 *      Author: prateek
 */

#ifndef CAMERA_H_
#define CAMERA_H_
#include <GLUT/glut.h>
#include <GL/glut.h>
#include <cmath>
#include <stdio.h>
using namespace std;

// Colors
//int motion=1;
// A camera.  It moves horizontally in a circle centered at the origin of
// radius 10.  It moves vertically straight up and down.

class Camera {
  double theta;      // determines the x and z positions
  double y;          // the current y position
  double dTheta;     // increment in theta for swinging the camera around
  double dy;         // increment in y for moving the camera up/down
public:
  Camera();
  double getX();
  double getY();
  double getZ();
  void moveRight();
  void moveLeft();
  void moveUp();
  void moveDown();
};

#endif /* CAMERA_H_ */
