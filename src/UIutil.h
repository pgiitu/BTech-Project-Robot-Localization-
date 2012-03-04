/*
 * UIutil.h
 *
 *  Created on: Jan 27, 2012
 *      Author: ashwani
 */

#ifndef UIUTIL_H_
#define UIUTIL_H_

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "Majoritymap.h"
#include "constants.h"
#include "Camera.h"

//void special(int key, int, int);
void init(void);
void errorCallback(GLenum errorCode);
void endCallback(void);
void beginCallback(GLenum which);
GLuint tessellate1(Polygon& polygon,int translate);
GLuint createlistHypothesis(std::list<Point> p);
GLuint createDisplayListmmap(Majoritymap mmap);








#endif /* UIUTIL_H_ */
