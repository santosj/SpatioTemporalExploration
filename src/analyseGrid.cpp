#include "CFremenGrid.h"
#include "CTimer.h"

#define MIN_X  -15.5
#define MIN_Y  -6.0
#define MIN_Z  -0.0
#define DIM_X 160
#define DIM_Y 120
#define DIM_Z 30 
#define RESOLUTION 0.1

using namespace std;

CFremenGrid *grid;
 
int main(int argc,char *argv[])
{
    grid = new CFremenGrid(MIN_X,MIN_Y,MIN_Z,DIM_X,DIM_Y,DIM_Z,RESOLUTION);
    grid->loadHeader(argv[1]);
    printf("3D Grid of %ix%ix%i loaded - %f information.\n",grid->xDim,grid->yDim,grid->zDim,grid->getObtainedInformation());
    delete grid;
    return 0;
}
