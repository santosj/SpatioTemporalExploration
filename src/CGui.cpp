#include "CGui.h"

#define THICK_CROSS

CGui::CGui(int wi,int he,int sc)
{
	height = he/sc;
	width = wi/sc;
	scale = sc;
	SDL_Init(SDL_INIT_VIDEO|SDL_HWSURFACE|SDL_HWACCEL);
	if(TTF_Init() == -1)printf("Unable to initialize SDL_ttf: %s\n", TTF_GetError());
	screen = NULL;
	screen = SDL_SetVideoMode(wi/sc,he/sc,24,SDL_HWSURFACE); 
	if (screen == NULL)fprintf(stderr,"Couldn't set SDL video mode: %s\r\n",SDL_GetError());
	SDL_WM_SetCaption("Robot revue vision system","Robot revue vision system");
	smallFont =  TTF_OpenFont("font.ttf",16);
	if(!smallFont){
		printf("Unable to open font: %s\n", TTF_GetError());
	}else{
		TTF_SetFontStyle(smallFont, TTF_STYLE_NORMAL);
	}
}

CGui::~CGui()
{
	SDL_FreeSurface(screen);
}

void CGui::drawImage(CRawImage* image)
{
	CRawImage *imageSrc = image;

	if (scale != 1){
		int wi = width;
		int he = height;
		imageSrc = new CRawImage(wi,he);
		for (int j = 0;j<he;j++){
			int srp = (j*scale)*wi*scale*3;
			int dep = j*wi*3;
			for (int i = 0;i<wi;i++){
				int dp = dep + i*3;
				int sp = srp + scale*i*3;
				imageSrc->data[dp] = image->data[sp];
				imageSrc->data[dp+1] = image->data[sp+1];
				imageSrc->data[dp+2] = image->data[sp+2];
			}
		}
	}
	SDL_Surface *imageSDL = SDL_CreateRGBSurfaceFrom(imageSrc->data,imageSrc->width,imageSrc->height,imageSrc->bpp*8,imageSrc->bpp*imageSrc->width,0x000000ff,0x0000ff00,0x00ff0000,0x00000000);
	//	SDL_Surface *imageSDL = SDL_ScaleSurface(imageSDL, width/scale, height/scale);
	if (imageSDL != NULL) SDL_BlitSurface(imageSDL, NULL, screen, NULL);
	//	SDL_FreeSurface(imageSDLa);
	SDL_FreeSurface(imageSDL);
	if (scale !=1) delete imageSrc;
}



void CGui::update()
{
  SDL_UpdateRect(screen,0,0,0,0);
}
