/*
 * UIutil.cpp
 *
 *  Created on: Jan 27, 2012
 *      Author: ashwani
 */

#include "UIutil.h"

void beginCallback(GLenum which)
{
   glBegin(which);
}

void endCallback(void)
{
   glEnd();
}

void errorCallback(GLenum errorCode)
{
   const GLubyte *estring;

   estring = gluErrorString(errorCode);
   fprintf (stderr, "Tessellation Error: %s\n", estring);
   exit (0);
}


void init(void)
{
		/*
		initialize viewing values */
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(-15,15.0,-15.0,15.0);
		//glOrtho(0.0, 500.0, 0.0, 0.0, 500.0, 0.0);
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT);
}

GLuint tessellate1(Polygon& polygon,int translate)
{
	GLuint id = glGenLists(1);  // create a display list
    if(!id) return id;          // failed to create a list, return 0

    GLUtesselator *tess = gluNewTess(); // create a tessellator
    if(!tess) return 0;  // failed to create tessellation object, return 0

    int size=polygon.size();
    GLdouble polygonArray[size][3];

    int i=0;
    for(VertexIterator vi = polygon.vertices_begin(); vi != polygon.vertices_end(); ++vi)
        {
    	        polygonArray[i][0]=translate+(*vi).cartesian(0);
    	        polygonArray[i][1]=(*vi).cartesian(1);
    	        polygonArray[i][2]=0;
    	        i++;
        }

    // register callback functions
    gluTessCallback(tess, GLU_TESS_VERTEX,(GLvoid (*) ()) &glVertex3dv);
    gluTessCallback(tess, GLU_TESS_BEGIN, (GLvoid (*) ()) &beginCallback);
    gluTessCallback(tess, GLU_TESS_END, (GLvoid (*) ()) &endCallback);
    gluTessCallback(tess, GLU_TESS_ERROR, (GLvoid (*) ()) &errorCallback);

    glNewList(id, GL_COMPILE);
	glColor3f(100,100,100);

	gluTessBeginPolygon(tess, NULL);                   // with NULL data
	   gluTessBeginContour(tess);
	   	   for(int j=0;j<size;j++)
	   	   	   {
	   	    		   gluTessVertex(tess, polygonArray[j], polygonArray[j]);
	   	 	   }
	   gluTessEndContour(tess);
    gluTessEndPolygon(tess);
    glEndList();

    gluDeleteTess(tess);        // delete after tessellation


    return id;      // return handle ID of a display list

}
