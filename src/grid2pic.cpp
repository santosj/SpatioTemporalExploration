using namespace std;

#include <ros/ros.h>
#include "CFremenGrid.h"
#include <iostream>
#include <cstdlib>
#include <fcntl.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL/SDL.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define MIN_X  -15.5
#define MIN_Y  -6.0
#define MIN_Z  -0.0
#define DIM_X 160
#define DIM_Y 120
#define DIM_Z 30 
#define RESOLUTION 0.1

CFremenGrid *grid;
SDL_Surface *screen;
static GLint T0 = 0;
static GLint Frames = 0;
int tranX = 0;
int tranY = 0;
GLdouble *vertices,*normals,*texCoords;
GLdouble modelMatrix[16],projMatrix[16];

//static GLfloat view_rotx = 20.0, view_roty = 30.0, view_rotz = 0.0;
static GLfloat angle = 0.0;

int width = 1920;
int height = 1080;

void draw(float phi);

static void draw(double ratio,float phi)
{
    glClearColor(1.0,1.0,1.0,1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPushMatrix();
    draw(phi);
    glPopMatrix();
    SDL_GL_SwapBuffers();
    Frames++;
    {
        GLint t = SDL_GetTicks();
        if (t - T0 >= 5000)
        {
            GLfloat seconds = (t - T0) / 1000.0;
            GLfloat fps = Frames / seconds;
            printf("%d frames in %g seconds = %g FPS\n", Frames, seconds, fps);
            T0 = t;
            Frames = 0;
        }
    }
}

void resizeGL( int w, int h )
{
  glViewport( 0, 0, w, h );
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  //glFrustum( -1920.0*2/470, 1920.0*2/470, -540.0*2/470, +540.0*2/470, 1.0, 25000.0 );
  glFrustum( -960.0/470, 960.0/470, -540.0/470, +540.0/470, 0.5, 25000.0 );
//  glFrustum( -540.0/470, 540.0/470, -1920.0/470, +1920.0/470, 0.5, 25000.0 );
}

static void
idle(void)
{
    angle += 2.0;
}

static void
init()
{
//    vertices = (GLdouble*) calloc(NUM_VERTICES,sizeof(GLdouble));
//    normals = (GLdouble*) calloc(NUM_VERTICES,sizeof(GLdouble));
    glShadeModel( GL_SMOOTH );
    glEnable(GL_DEPTH_TEST);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 0);
    GLfloat ambientLight[] = {0.1,0.1,0.1,0.1};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,ambientLight);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);

}

void drawCube(float x,float y, float z)
{
	glPushMatrix();
	glTranslatef(-x*1000,-y*1000,z*1000);
	glLineWidth( 1.0 );
	GLfloat redBatteryColor[] = {   z*0.5,1.0-z*0.5,  0.00};
	glColor3fv(redBatteryColor);
	glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,redBatteryColor);
	glBegin(GL_POLYGON);
	glColor3f(   z*0.5,1.0-z*0.5,  0.00);
	glNormal3f(+1.0,-1.0,+1.0);
	glVertex3f(  50, -50, 50 );
	glNormal3f(+1.0,+1.0,+1.0);
	glVertex3f(  50,  50, 50 );
	glNormal3f(-1.0,+1.0,+1.0 );
	glVertex3f( -50,  50, 50 );
	glNormal3f(-1.0,-1.0,+1.0 );
	glVertex3f( -50, -50, 50 );
	glEnd();

	glBegin(GL_POLYGON);
	glNormal3f(+1.0,-1.0,-1.0);
	glVertex3f( 50, -50, -50 );
	glNormal3f(+1.0,+1.0,-1.0);
	glVertex3f( 50,  50, -50 );
	glNormal3f(+1.0,+1.0,+1.0);
	glVertex3f( 50,  50,  50 );
	glNormal3f(+1.0,-1.0,+1.0);
	glVertex3f( 50, -50,  50 );
	glEnd();

	glBegin(GL_POLYGON);
	glNormal3f(-1.0,-1.0,+1.0);
	glVertex3f( -50, -50,  50 );
	glNormal3f(-1.0,+1.0,+1.0);
	glVertex3f( -50,  50,  50 );
	glNormal3f(-1.0,+1.0,-1.0);
	glVertex3f( -50,  50, -50 );
	glNormal3f(-1.0,-1.0,-1.0);
	glVertex3f( -50, -50, -50 );
	glEnd();

	glBegin(GL_POLYGON);
	glNormal3f(+1.0,+1.0,+1.0);
	glVertex3f(  50,  50,  50 );
	glNormal3f(+1.0,+1.0,-1.0);
	glVertex3f(  50,  50, -50 );
	glNormal3f(-1.0,+1.0,-1.0);
	glVertex3f( -50,  50, -50 );
	glNormal3f(-1.0,+1.0,+1.0);
	glVertex3f( -50,  50,  50 );
	glEnd();

	glBegin(GL_POLYGON);
	glNormal3f(+1.0,-1.0,-1.0);
	glVertex3f(  50, -50, -50 );
	glNormal3f(+1.0,-1.0,+1.0);
	glVertex3f(  50, -50,  50 );
	glNormal3f(-1.0,-1.0,+1.0);
	glVertex3f( -50, -50,  50 );
	glNormal3f(-1.0,-1.0,-1.0);
	glVertex3f( -50, -50, -50 );
	glEnd();

	glPopMatrix();
}

void draw(float phi)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	//glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
//	glRotatef( phi , 0.0, 1.0, 0.0);
//	glRotatef( 90.0 , 1.0, 0.0, 0.0);
	glTranslatef(-8000+tranX,-1000+tranY,-2750);
	glRotatef( 0.0 , 0.0, 0.0, 1.0);
	//glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
	//glMatrixMode( GL_MODELVIEW );


	float resolution = grid->resolution; 
	float minX = grid->oX;
	float minY = grid->oY;
	float minZ = grid->oZ;
	float maxX = minX+grid->xDim*grid->resolution-3*grid->resolution/4;
	float maxY = minY+grid->yDim*grid->resolution-3*grid->resolution/4;
	float maxZ = 2.1;//minZ+grid->zDim*grid->resolution-3*grid->resolution/4;
	int cnt = 0;
	int cells = 0;
	int type = 1;
	int stamp = 1000000000;
	float period = 0;
	float estimate,minP,maxP;
	minP = 0.4; 
	maxP = 1.0; 
	if (stamp != 0 && type == 1) grid->recalculate(stamp);

	//iterate over the cells' probabilities 
	for(float z = minZ;z<maxZ;z+=resolution){
		for(float y = minY;y<maxY;y+=resolution){
			for(float x = minX;x<maxX;x+=resolution){
				if (type == 0) estimate = grid->retrieve(cnt);			//short-term memory grid
				if (type == 1) estimate = grid->estimate(cnt,0);			//long-term memory grid
				if (type == 2) estimate = grid->aux[cnt];				//auxiliary grid
				if (type == 3) estimate = grid->getDominant(cnt,period);	//dominant frequency amplitude
				
				if(estimate > minP && estimate < maxP)
				{
					drawCube(x,y,z);
					cells++;
				}
				cnt++;
			}
		}
	}

}

void distortView(unsigned char* dst, unsigned char* src)
{
	int pos = 0;
	int posb = 0;
	float xp,yp,r,xpr,ypr;
	float fx,fy,cx,cy;
	fx = 470;
	fy = 470;
	cx = 960;
	cy = 540;
	float kc[] = {0,-0.14,0.013,0.0001735,0.002564,0,0}; 
	memset(dst,0,1920*1080*3);
	for (int y = 0;y<1080;y++){
		yp = (y-cy)/fy;
		for (int x = 0;x<1980;x++){
			pos = 3*(1920*y+x);
			xp = (x-cx)/fx*2;
			yp = (y-cy)/fy*2;
			r = xp*xp+yp*yp;
			float dpx = 2*kc[3]*xp*yp + kc[4]*(r + 2*xp*xp);
			xpr = (1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r)*xp+dpx;

			float dpy = 2*kc[4]*xp*yp + kc[3]*(r + 2*yp*yp);
			ypr = (1+kc[1]*r+kc[2]*r*r+kc[5]*r*r*r)*yp+dpy;

			xp = cx + fx*xpr;
			yp = cy + fy*ypr;
			if (xp > 0 && xp <1920 && yp > 0 && yp < 1080){
				posb = 3*(1920*((int)yp)+(int)xp);
				dst[posb+0] = src[pos+0];
				dst[posb+1] = src[pos+1];
				dst[posb+2] = src[pos+2];
			}
		}
		printf("%i\n",y);
	}
	printf("FIN\n");
}

void saveScreen(const char *filename)
{
	SDL_Surface *image;
	SDL_Surface *temp;
	int idx;
	int w = screen->w;
	int h = screen->h;

	image = SDL_CreateRGBSurface(SDL_SWSURFACE, w, h, 24, 0x0000FF, 0x00FF00, 0xFF0000, 0x000000);
	temp = SDL_CreateRGBSurface(SDL_SWSURFACE, w, h, 24, 0x0000FF, 0x00FF00, 0xFF0000, 0x000000);
	glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, image->pixels);
	for (idx = 0; idx < h; idx++)
	{
		memcpy((unsigned char *)(temp->pixels) + 3 * w * idx,
				(unsigned char *)(image->pixels) + 3 * w * (h - idx),
				3*w);
	}
	distortView((unsigned char*)(image->pixels),(unsigned char*)(temp->pixels));

	SDL_SaveBMP(image, filename);
	SDL_FreeSurface(image);
	SDL_FreeSurface(temp);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "fremenutil");
	ros::NodeHandle n;
	grid = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);
	grid->load(argv[1]);

	int done;

	SDL_Init(SDL_INIT_VIDEO);
	screen = SDL_SetVideoMode(width, height, 24, SDL_OPENGL|SDL_HWSURFACE);

	init();
	resizeGL(screen->w, screen->h);
	done = 0;
	SDL_Event event;
	int cycle = 0;
	while ( ! done )
	{
		idle();
		SDL_PollEvent(&event);
		Uint8 *keys;
		keys = SDL_GetKeyState(NULL);
		//keyPressEvent(keys);
		if (keys[SDLK_ESCAPE]) done = 1;
		if (keys[SDLK_LEFT]) tranX+=100; 
		if (keys[SDLK_RIGHT]) tranX-=100; 
		if (keys[SDLK_UP]) tranY-=100; 
		if (keys[SDLK_DOWN]) tranY+=100; 
		draw(screen->h/screen->w,atof(argv[2]));
		if (keys[SDLK_s]) saveScreen("aha.bmp");
		printf("%i\n",cycle++);
	}
	SDL_Quit();
	return 0;
}
