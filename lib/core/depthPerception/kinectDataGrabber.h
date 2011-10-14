/*
 * kinectDataGrabber.h
 *
 *  Created on: Oct 11, 2011
 *      Author: reon
 */

#ifndef KINECTDATAGRABBER_H_
#define KINECTDATAGRABBER_H_

#include "libfreenect.h"
#include "libfreenect_sync.h"

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

namespace BRICS_3D {

class kinectDataGrabber {

private:

	/**
	 * Calibrated RGB values of the depth-points
	 */
    char RGBvalues[480][640][3];


    /**
     * Depth values of the points
     */
    static short xyz[480][640][3];

	// Do the projection from u,v,depth to X,Y,Z directly in an opengl matrix
	// These numbers come from a combination of the ros kinect_node wiki, and
	// nicolas burrus' posts.
	void LoadVertexMatrix()
	{
	    float fx = 594.21f;
	    float fy = 591.04f;
	    float a = -0.0030711f;
	    float b = 3.3309495f;
	    float cx = 339.5f;
	    float cy = 242.7f;
	    GLfloat mat[16] = {
	        1/fx,     0,  0, 0,
	        0,    -1/fy,  0, 0,
	        0,       0,  0, a,
	        -cx/fx, cy/fy, -1, b
	    };
	    glMultMatrixf(mat);
	}


	// This matrix comes from a combination of nicolas burrus's calibration post
	// and some python code I haven't documented yet.
	void LoadRGBMatrix()
	{
	    float mat[16] = {
	        5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
	        -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
	        -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
	        -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02
	    };
	    glMultMatrixf(mat);
	}


	void no_kinect_quit(void)
	{
	    printf("Error: Kinect not connected?\n");
	    exit(1);
	}

	void DrawGLScene()
	{
	    short *depth = 0;
	    char *rgb = 0;
	    uint32_t ts;
	    GLuint gl_rgb_tex;
	    int rotangles[2] = {0}; // Panning angles
	    float zoom = 1;         // zoom factor



	    if (freenect_sync_get_depth((void**)&depth, &ts, 0, FREENECT_DEPTH_11BIT) < 0)
		no_kinect_quit();
	    if (freenect_sync_get_video((void**)&rgb, &ts, 0, FREENECT_VIDEO_RGB) < 0)
		no_kinect_quit();

	    static unsigned int indices[480][640];

	    int i,j;
	    for (i = 0; i < 480; i++) {
	        for (j = 0; j < 640; j++) {
	            xyz[i][j][0] = j;
	            xyz[i][j][1] = i;
	            xyz[i][j][2] = depth[i*640+j];
	            indices[i][j] = i*640+j;
	        }
	    }

	    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	    glLoadIdentity();

	    glPushMatrix();
	    glScalef(zoom,zoom,1);
	    glTranslatef(0,0,-3.5);
	    glRotatef(rotangles[0], 1,0,0);
	    glRotatef(rotangles[1], 0,1,0);
	    glTranslatef(0,0,1.5);

	    LoadVertexMatrix();

	    // Set the projection from the XYZ to the texture image
	    glMatrixMode(GL_TEXTURE);
	    glLoadIdentity();
	    glScalef(1/640.0f,1/480.0f,1);
	    LoadRGBMatrix();
	    LoadVertexMatrix();
	    glMatrixMode(GL_MODELVIEW);

	    glPointSize(1);

	    glEnableClientState(GL_VERTEX_ARRAY);
	    glVertexPointer(3, GL_SHORT, 0, xyz);
	    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	    glTexCoordPointer(3, GL_SHORT, 0, xyz);

//	    if (color)
	        glEnable(GL_TEXTURE_2D);
	    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb);

	    /**----------------------------------------------------
	     * Getting the rgb image
	     *@reon
	     */
	    glGetTexImage(	GL_TEXTURE_2D,0, GL_RGB, GL_UNSIGNED_BYTE, rgb);
	    for (i = 0; i < 480; i++) {
	    	j=0;
	        while(j<640*3){
	            RGBvalues[i][j][0] = rgb[i*640+j];
	        	j++;
	            RGBvalues[i][j][1] = rgb[i*640+j];
	            j++;
	            RGBvalues[i][j][2] = rgb[i*640+j];
	            j++;
	        }
	    }
	    //printf("rgb values are [%d, %d, %d]", RGBvalues[10][10][2],
	    	//	RGBvalues[11][11][2],RGBvalues[12][12][2]);

	    /**----------------------------------------------------@reon-end*/
//	    glPointSize(2.0f);
//	    glDrawElements(GL_POINTS, 640*480, GL_UNSIGNED_INT, indices);
//	    glPopMatrix();
//	    glDisable(GL_TEXTURE_2D);
//	    glutSwapBuffers();
	}

public:
	kinectDataGrabber();
	virtual ~kinectDataGrabber();
};

}

#endif /* KINECTDATAGRABBER_H_ */
