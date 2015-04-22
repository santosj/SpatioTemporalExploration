#ifndef __CGUI_H__
#define __CGUI_H__

#include "CRawImage.h"
#include <math.h>
#include <SDL/SDL.h>
#include <SDL/SDL_ttf.h>

class CGui
{
public:
  CGui(int wi,int he,int scale);
  ~CGui();

  void drawImage(CRawImage* image);
  void update();

private:
  SDL_Surface *screen;
  int width,height,scale;
  TTF_Font *smallFont;
};

#endif

/* end of CGui.h */
