//////////////////////////////////////////////
//////////////////////////////////////////////
///PSP ODE Example - Anonymous Tipster////////
//////////////////////////////////////////////
///  This Example shows how to use my port ///
///of the ODE physics engine (Russel Smith)///
///  The world is setup as an infinite     ///
///plane, and then 100 cubes are dropped   ///
///onto each other.                        ///
///  Use Arrow Keys to spin around the     ///
///plane and zoom in. The RTrigger can be  ///
///used to pause and unpause the simulation///
///  Source code based on PSPSDK Cube      ///
///example, and uses code from spharm for  ///
///loading textures.                       ///
//////////////////////////////////////////////
//////////////////////////////////////////////
/// N.B. LicenseBSD must be included with  ///
///applications that use the ODE engine, as///
///the creator of ODE requires             ///
//////////////////////////////////////////////
//////////////////////////////////////////////

#include <pspkernel.h>
#include <pspiofilemgr.h>
#include <pspdisplay.h>
#include <pspdebug.h>
#include <pspctrl.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <malloc.h>
#include <psppower.h>

#include <pspgu.h>
#include <pspge.h>
#include <pspgum.h>

#include "ode/ode.h"

 
//#ifdef __cplusplus
//extern "C" {
//#endif

PSP_MODULE_INFO("ODE Test", 0, 1, 1);
PSP_MAIN_THREAD_ATTR(THREAD_ATTR_USER);

#define printf	pspDebugScreenPrintf

static unsigned int __attribute__((aligned(16))) list[262144];
extern unsigned char logo_start[];

//values for the camera path
float circleCamX, circleCamY;

//textures
extern unsigned char brickTex_start[];
extern unsigned char metalRibbed_start[];
extern unsigned char metalRibbedMedium_start[];

unsigned char *brickTex_temp; 
unsigned char *metalRibbed_temp; 
unsigned char *metalRibbedMedium_temp; 

struct Vertex
{
	float u, v;
	float nx,ny,nz;
	float x,y,z;
};

#define CUBESIZE 2

struct Vertex vertices[12*3] =
{
	{0, 0,  0,0,1,-(CUBESIZE/2),-(CUBESIZE/2), (CUBESIZE/2)}, // 0
	{1, 0,  0,0,1,-(CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 4
	{1, 1,  0,0,1, (CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 5

	{0, 0,  0,0,1,-(CUBESIZE/2),-(CUBESIZE/2), (CUBESIZE/2)}, // 0
	{1, 1,  0,0,1, (CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 5
	{0, 1,  0,0,1, (CUBESIZE/2),-(CUBESIZE/2), (CUBESIZE/2)}, // 1

	{0, 0,  0,0,-1,-(CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 3
	{1, 0,  0,0,-1, (CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 2
	{1, 1,  0,0,-1, (CUBESIZE/2), (CUBESIZE/2),-(CUBESIZE/2)}, // 6

	{0, 0,  0,0,-1,-(CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 3
	{1, 1,  0,0,-1, (CUBESIZE/2), (CUBESIZE/2),-(CUBESIZE/2)}, // 6
	{0, 1,  0,0,-1,-(CUBESIZE/2), (CUBESIZE/2),-(CUBESIZE/2)}, // 7

	{0, 0,  1,0,0, (CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 0
	{1, 0,  1,0,0, (CUBESIZE/2),-(CUBESIZE/2), (CUBESIZE/2)}, // 3
	{1, 1,  1,0,0, (CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 7

	{0, 0,  1,0,0, (CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 0
	{1, 1,  1,0,0, (CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 7
	{0, 1,  1,0,0, (CUBESIZE/2), (CUBESIZE/2),-(CUBESIZE/2)}, // 4

	{0, 0,  -1,0,0,-(CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 0
	{1, 0,  -1,0,0,-(CUBESIZE/2), (CUBESIZE/2),-(CUBESIZE/2)}, // 3
	{1, 1,  -1,0,0,-(CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 7

	{0, 0,  -1,0,0,-(CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 0
	{1, 1,  -1,0,0,-(CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 7
	{0, 1,  -1,0,0,-(CUBESIZE/2),-(CUBESIZE/2), (CUBESIZE/2)}, // 4

	{0, 0,  0,1,0,-(CUBESIZE/2), (CUBESIZE/2),-(CUBESIZE/2)}, // 0
	{1, 0,  0,1,0, (CUBESIZE/2), (CUBESIZE/2),-(CUBESIZE/2)}, // 1
	{1, 1,  0,1,0, (CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 2

	{0, 0,  0,1,0,-(CUBESIZE/2), (CUBESIZE/2),-(CUBESIZE/2)}, // 0
	{1, 1,  0,1,0, (CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 2
	{0, 1,  0,1,0,-(CUBESIZE/2), (CUBESIZE/2), (CUBESIZE/2)}, // 3

	{0, 0,  0,-1,0,-(CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 4
	{1, 0,  0,-1,0,-(CUBESIZE/2),-(CUBESIZE/2), (CUBESIZE/2)}, // 7
	{1, 1,  0,-1,0, (CUBESIZE/2),-(CUBESIZE/2), (CUBESIZE/2)}, // 6

	{0, 0,  0,-1,0,-(CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 4
	{1, 1,  0,-1,0, (CUBESIZE/2),-(CUBESIZE/2), (CUBESIZE/2)}, // 6
	{0, 1,  0,-1,0, (CUBESIZE/2),-(CUBESIZE/2),-(CUBESIZE/2)}, // 5
};

#define FLOORSIZE 6

struct Vertex floorVert[2*3] =
{
	{1, 1,0,1,0, (FLOORSIZE/2),0, (FLOORSIZE/2)}, // 6
	{1, 0,0,1,0,-(FLOORSIZE/2),0, (FLOORSIZE/2)}, // 7
	{0, 0,0,1,0,-(FLOORSIZE/2),0,-(FLOORSIZE/2)}, // 4
	

	{0, 1,0,1,0, (FLOORSIZE/2),0,-(FLOORSIZE/2)}, // 5
	{1, 1,0,1,0, (FLOORSIZE/2),0, (FLOORSIZE/2)}, // 6
	{0, 0,0,1,0,-(FLOORSIZE/2),0,-(FLOORSIZE/2)}, // 4
	
};

#define floorBlocksX 10
#define floorBlocksY 10

int SetupCallbacks();

//physics specific stuff
// some constants

#define NUM 100			// number of bodies
#define SIDE (0.2)		// side length of a box
#define MASS (1.0)		// mass of a box
#define MAX_CONTACTS 4		// maximum number of contact points per body
#define GRAVITY -9.8

//These defines set how easily the objects will be
//told to disable (aka sleeping), so they don't consume CPU time
#define LINEAR_DISABLE_THRESHOLD 0.1
#define ANGULAR_DISABLE_THRESHOLD 0.1
//how many steps the body must satisfy the Thresholds for it to be disabled - higher for a more stable simulation
#define STEPS_FOR_DISABLE 15

// dynamics and collision objects

static dWorldID world=0;
static dBodyID body[NUM];
static dGeomID bodyGeom[NUM];

static dGeomID floorPhys;
static dSpaceID space;
static dJointGroupID contactgroup;//used in collision-joints

int advanceSimulation = 1;//pause or advance simulation?
int pauseCount = 0;//counts up to stop the simulation pausing and unpausing too fast



#define BUF_WIDTH (512)
#define SCR_WIDTH (480)
#define SCR_HEIGHT (272)
#define PIXEL_SIZE (4) /* change this if you change to another screenmode */
#define FRAME_SIZE (BUF_WIDTH * SCR_HEIGHT * PIXEL_SIZE)
#define ZBUF_SIZE (BUF_WIDTH SCR_HEIGHT * 2) /* zbuffer seems to be 16-bit? */


void init();
unsigned char *convertimage(unsigned char *inptr,int size);
void HandleKeysGame();
//helper functions
const ScePspFMatrix4 getPspMatrixFromBody(dBodyID body);
const ScePspFMatrix4 getPspMatrixFromGeom(dGeomID geom);


static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	//In ODE, whenever there is a collision, you need to create a joint
	//between the two objects which have collided, you then advance the simulation
	//and disconnect all the joints, ready for next time.

	//This function creates the joints we need

  int i;
  // if (o1->body && o2->body) return;

  // exit without doing anything if the two bodies are connected by a joint
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;


  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM;
    contact[i].surface.mu = dInfinity;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = 0.1;
    contact[i].surface.bounce_vel = 0.09;
    contact[i].surface.soft_cfm = 0.001;
  }
  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			   sizeof(dContact))) {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};
    for (i=0; i<numc; i++) {
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,b1,b2);
      //if (show_contacts) dsDrawBox (contact[i].geom.pos,RI,ss);
    }
  }
}

void init(){
	pspDebugScreenInit();
	circleCamX=50; 
	circleCamY=50;
//setup images
	
	brickTex_temp = convertimage(brickTex_start,32);
	metalRibbed_temp = convertimage(metalRibbed_start,64);
	metalRibbedMedium_temp = convertimage(metalRibbedMedium_start,32);

//setup physics world
	
	if (world) dWorldDestroy (world);
	world = dWorldCreate();//create the world, which houses all our bodies
	space = dHashSpaceCreate (0);//create the 'space', which manages geoms and collisions
	contactgroup = dJointGroupCreate (0);//create the joints used in the collision callback
	dWorldSetERP (world,1.0);
	dWorldSetCFM (world,0.0);
    dWorldSetAutoDisableFlag (world,1);
	dWorldSetAutoDisableLinearThreshold(world, LINEAR_DISABLE_THRESHOLD);
	dWorldSetAutoDisableAngularThreshold(world, ANGULAR_DISABLE_THRESHOLD);
	dWorldSetAutoDisableTime(world,0);
	dWorldSetAutoDisableSteps(world,STEPS_FOR_DISABLE);
    dWorldSetContactMaxCorrectingVel (world,100.0);
    dWorldSetContactSurfaceLayer (world,0.000);
	dWorldSetGravity (world,0,GRAVITY,0); 


	// create bodies

	//Included are two modes for setting up the blocks
	//A default tower mode, where the blocks are put
	//one on top of each other.
	//And a wall mode, where the blocks are organised
	//into a wall. This second mode does not work corretly
	//under current parameters, the blocks jitter and fall
	//over, but the code is included for testing/learning
	//purposes.
	
	/////////////Create tower
	int i,j;
  for (i=0; i<NUM; i++) {
    // create bodies at random position and orientation
    body[i] = dBodyCreate (world);
    dBodySetPosition (body[i],0,5+(i*3),0);
	
    dReal q[4];
    for (j=0; j<4; j++) q[j] = dRandReal()/1000;
    dBodySetQuaternion (body[i],q);
    // set random velocity
    dBodySetLinearVel (body[i], 0,0, 0);
    dBodySetAngularVel (body[i], 0,0, 0);

    // set random mass (random diagonal mass rotated by a random amount)
    dMass m;
    dMatrix3 R;
    dMassSetBox (&m,1,CUBESIZE,CUBESIZE,CUBESIZE);
    dMassAdjust (&m,dRandReal()+1);
    for (j=0; j<4; j++) q[j] = dRandReal()/1000;
    dQtoR (q,R);
    dMassRotate (&m,R);
    dBodySetMass (body[i],&m);

	bodyGeom[i] = dCreateBox (space,CUBESIZE,CUBESIZE,CUBESIZE);
    dGeomSetBody (bodyGeom[i],body[i]);
  }
  //////////////end create tower

	/*
	/////////////create wall
	int x,y,i,j;
	int widthStack = 20;
	i=0;
  for (x=0; x<widthStack; x++) {
	  for (y=0; y<NUM/widthStack; y++) {
		  //i=x*y;
    // create bodies at random position and orientation
    body[i] = dBodyCreate (world);
		

    dBodySetPosition (body[i],(x*2.001)-(widthStack*CUBESIZE/2),1+(y*2.001),0);
	//dBodySetPosition (body[i],0,i*3,0);
	
    //dReal q[4];
    //for (j=0; j<4; j++) q[j] = 0.00001;
    //dBodySetQuaternion (body[i],q);
    // set random velocity
    dBodySetLinearVel (body[i], 0,0, 0);
    dBodySetAngularVel (body[i], 0,0, 0);

    // set random mass (random diagonal mass rotated by a random amount)
    dMass m;
    dMatrix3 R;
    dMassSetBox (&m,1,CUBESIZE,CUBESIZE,CUBESIZE);
    dMassAdjust (&m,0.001);
    //for (j=0; j<4; j++) q[j] = 0;
    //dQtoR (q,R);
    //dMassRotate (&m,R);
    dBodySetMass (body[i],&m);

	bodyGeom[i] = dCreateBox (space,CUBESIZE,CUBESIZE,CUBESIZE);
    dGeomSetBody (bodyGeom[i],body[i]);
	dBodyDisable(body[i]);
	i++;
  }
 }
 //////////////////end create wall
  */


	//create floor
  floorPhys = dCreatePlane (space,0,1,0,0);
  //dSpaceAdd(space,floorPhys);

}

int main(int argc, char* argv[])
{
	SetupCallbacks();
	init();

	// setup GU

	sceGuInit();

	sceGuStart(GU_DIRECT,list);
	sceGuEnable(GU_CLIP_PLANES);
	sceGuDrawBuffer(GU_PSM_8888,(void*)0,BUF_WIDTH);
	sceGuDispBuffer(SCR_WIDTH,SCR_HEIGHT,(void*)0x88000,BUF_WIDTH);
	sceGuDepthBuffer((void*)0x110000,BUF_WIDTH);
	sceGuOffset(2048 - (SCR_WIDTH/2),2048 - (SCR_HEIGHT/2));
	sceGuViewport(2048,2048,SCR_WIDTH,SCR_HEIGHT);
	sceGuDepthRange(0xc350,0x2710);
	sceGuScissor(0,0,SCR_WIDTH,SCR_HEIGHT);
	sceGuEnable(GU_SCISSOR_TEST);
	sceGuDepthFunc(GU_GEQUAL);
	//sceGuAlphaFunc(GU_GREATER,0,0xff);
	//sceGuEnable(GU_ALPHA_TEST);
	sceGuEnable(GU_DEPTH_TEST);
	sceGuFrontFace(GU_CW);
	sceGuShadeModel(GU_SMOOTH);
	sceGuEnable(GU_CULL_FACE);
	sceGuEnable(GU_TEXTURE_2D);
	//lights aren't currently defined, try adding them ^_^
	sceGuEnable(GU_LIGHTING);
	sceGuEnable(GU_LIGHT0);
	sceGuEnable(GU_LIGHT1);
	sceGuEnable(GU_LIGHT2);
	sceGuEnable(GU_LIGHT3);
	sceGuDisable(GU_ALPHA_TEST);
	//sceGuEnable(GU_UNKNOWN_17);//activate color-keying
	//sceGuEnable(GU_BLEND);
	sceGuFinish();
	sceGuSync(0,0);	
	sceKernelDcacheWritebackAll();
	sceDisplayWaitVblankStart();
	sceGuDisplay(GU_TRUE);

	sceDisplayWaitVblankStart();
	sceGuDisplay(GU_TRUE);

	float time0 = 0.0;
	float time1 = 0.0;
	time0= clock();
	time1= clock();
	float frameLength = 0.0;
	
	// run sample

	int val = 0;

	for(;;)
	{

		sceGuStart(GU_DIRECT,list);

		// clear screen

		sceGuClearColor(0xff554433);
		sceGuClearDepth(0);
		sceGuClear(GU_COLOR_BUFFER_BIT|GU_DEPTH_BUFFER_BIT);

		// setup matrices for cube

		sceGumMatrixMode(GU_PROJECTION);
		sceGumLoadIdentity();
		sceGumPerspective(50.0f,16.0f/9.0f,0.5f,1000.0f);

		HandleKeysGame();

		//////////////////////
		////UPDATE PHYSICS////
		//////////////////////
		dSpaceCollide (space,0,&nearCallback);//we need to call the collide function to setup temp joints
		time1 = clock();
		//advance the simulation - if the time to be passed is a bit crazy, go for a 0.05
		if(advanceSimulation){
		if((time1-time0)/1000000 < 0.1){dWorldQuickStep(world,(time1-time0)/1000000);}else{dWorldQuickStep(world,0.05);}
			//dWorldQuickStep(world,0.05);//<-- framerate dependant
		}
		// remove all contact joints, ready for next time
		dJointGroupEmpty (contactgroup);
		pspDebugScreenSetXY(0,0);
		printf("Frame: %f", time1-time0);
		pspDebugScreenSetXY(0,1);
		printf("FPS: %f", 1/((time1-time0)/1000000));
		pspDebugScreenSetXY(0,2);
		printf("Arrow Keys To move camera");
		pspDebugScreenSetXY(0,3);
		printf("RTrigger to pause/unpause");
		pspDebugScreenSetXY(0,4);
		printf("AnonymousTipster's ODE port");
		pspDebugScreenSetXY(0,5);
		printf("Based on Russel Smith's ODE");


		time0 = clock();

		sceGumMatrixMode(GU_VIEW);
		sceGumLoadIdentity();
		{
			
		ScePspFVector3 cameraPos = { 0, 4, -10};
		ScePspFVector3 lookAtPos = { 0, 0, 0};
		ScePspFVector3 upVec = { 0, 1, 0};
		upVec.y = 1;
		
	float awayFrom;//distance from center to camera
	awayFrom  = (circleCamY/50)*20.0f;
	float pointX;
	float pointY;
	float angle;
	angle = (circleCamX/50)*6.28;
	pointX = 0+sin(angle)*awayFrom;
	pointY = 0+cos(angle)*awayFrom;

	int pointXi, pointYi;
	pointXi = pointX;pointYi = pointY;

	cameraPos.x = pointX;
	cameraPos.y = 5.0f;
	cameraPos.z = pointY;

	sceGumLookAt(&cameraPos, &lookAtPos, &upVec);
		}


		//draw floor tiles
		// setup texture
		sceGuTexMode(GU_PSM_8888,1,0,0);
		sceGuTexImage(0,64,64,64,metalRibbed_temp);
		sceGuTexImage(1,32,32,32,metalRibbedMedium_temp);
		sceGuTexFunc(GU_TFX_REPLACE,GU_TCC_RGB);
		sceGuTexEnvColor(0xffff00);
		sceGuTexFilter(GU_LINEAR_MIPMAP_LINEAR,GU_LINEAR_MIPMAP_LINEAR);
		sceGuTexScale(1.0f,1.0f);
		sceGuTexOffset(0.0f,0.0f);
		sceGuAmbientColor(0xffffffff);
		int x,y;
		for(x=0;x<floorBlocksX;x++){
			for(y=0;y<floorBlocksY;y++){
			sceGumMatrixMode(GU_MODEL);
		sceGumLoadIdentity();
		{
			ScePspFVector3 pos = {-(floorBlocksX*FLOORSIZE/2)+(x*FLOORSIZE), 0, -(floorBlocksY*FLOORSIZE/2)+(y*FLOORSIZE) };
			ScePspFVector3 rot = { 0, 0, 0};
			sceGumRotateXYZ(&rot);
			sceGumTranslate(&pos);
		}
		// draw floor
		sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D,2*3,0,floorVert);

			}
		}
		
		
		//draw cube objects
		// setup texture
		sceGuTexMode(GU_PSM_8888,0,0,0);
		sceGuTexImage(0,32,32,32,brickTex_temp);
		sceGuTexFunc(GU_TFX_REPLACE,GU_TCC_RGB);
		sceGuTexEnvColor(0xffff00);
		sceGuTexFilter(GU_LINEAR,GU_LINEAR);
		sceGuTexScale(1.0f,1.0f);
		sceGuTexOffset(0.0f,0.0f);
		sceGuAmbientColor(0xffffffff);

		
			int i;
		for (i=0; i<NUM; i++) {
			//const dReal cubePosition;
			//cubePosition = dBodyGetPosition(body[i]);
		sceGumMatrixMode(GU_MODEL);
		sceGumLoadIdentity();

		/*
		{
			ScePspFVector3 pos = { dBodyGetPosition(body[i])[0], dBodyGetPosition(body[i])[1], dBodyGetPosition(body[i])[2] };
			ScePspFVector3 rot = { 0, 0, 0};
			sceGumRotateXYZ(&rot);
			sceGumTranslate(&pos);
		}*/
		sceGumLoadMatrix(&getPspMatrixFromBody(body[i]));
		// draw cube
		sceGumDrawArray(GU_TRIANGLES,GU_TEXTURE_32BITF|GU_NORMAL_32BITF|GU_VERTEX_32BITF|GU_TRANSFORM_3D,12*3,0,vertices);

		}


		sceGuFinish();
		sceGuSync(0,0);

		sceDisplayWaitVblankStart();
		sceGuSwapBuffers();
	}

	sceGuTerm();
	//unload physics world
	dJointGroupDestroy (contactgroup);//finished with joints FOREVER..so destroy them
	dSpaceDestroy (space);
	dWorldDestroy (world);

	sceKernelExitGame();
	return 0;
}

void HandleKeysGame(){
	
	SceCtrlData pad;

	sceCtrlSetSamplingCycle(0);
	sceCtrlSetSamplingMode(PSP_CTRL_MODE_ANALOG);
    sceCtrlReadBufferPositive(&pad, 1); 

	if(pad.Buttons & PSP_CTRL_UP){circleCamY--;}
	if(pad.Buttons & PSP_CTRL_DOWN){circleCamY++;}
	if(pad.Buttons & PSP_CTRL_LEFT){circleCamX-=0.3;}
	if(pad.Buttons & PSP_CTRL_RIGHT){circleCamX+=0.3;}

	if(pad.Buttons & PSP_CTRL_RTRIGGER && pauseCount > 8){if(advanceSimulation == 0){advanceSimulation = 1;pauseCount = 0;}else{advanceSimulation = 0;pauseCount = 0;}}
	pauseCount++;if(pauseCount > 10){pauseCount = 10;}

}

const ScePspFMatrix4 getPspMatrixFromBody(dBodyID body){
const dReal *pos;
const dReal *R;

pos = dBodyGetPosition(body);
R = dBodyGetRotation(body);

ScePspFMatrix4 matrix;

  matrix.x.x=R[0];
  matrix.x.y=R[4];
  matrix.x.z=R[8];
  matrix.x.w=0;
  matrix.y.x=R[1];
  matrix.y.y=R[5];
  matrix.y.z=R[9];
  matrix.y.w=0;
  matrix.z.x=R[2];
  matrix.z.y=R[6];
  matrix.z.z=R[10];
  matrix.z.w=0;
  matrix.w.x=pos[0];
  matrix.w.y=pos[1];
  matrix.w.z=pos[2];
  matrix.w.w=1;

  return matrix;

}

const ScePspFMatrix4 getPspMatrixFromGeom(dGeomID geom){
const dReal *pos;
const dReal *R;

pos = dGeomGetPosition(geom);
R = dGeomGetRotation(geom);

ScePspFMatrix4 matrix;

  matrix.x.x=R[0];
  matrix.x.y=R[4];
  matrix.x.z=R[8];
  matrix.x.w=0;
  matrix.y.x=R[1];
  matrix.y.y=R[5];
  matrix.y.z=R[9];
  matrix.y.w=0;
  matrix.z.x=R[2];
  matrix.z.y=R[6];
  matrix.z.z=R[10];
  matrix.z.w=0;
  matrix.w.x=pos[0];
  matrix.w.y=pos[1];
  matrix.w.z=pos[2];
  matrix.w.w=1;

  return matrix;

}

/* Exit callback */
int exit_callback(int arg1, int arg2, void *common)
{
	sceKernelExitGame();
	return 0;
}

/* Callback thread */
int CallbackThread(SceSize args, void *argp)
{
	int cbid;

	cbid = sceKernelCreateCallback("Exit Callback", exit_callback, NULL);
	sceKernelRegisterExitCallback(cbid);

	sceKernelSleepThreadCB();

	return 0;
}

/* Sets up the callback thread and returns its thread id */
int SetupCallbacks(void)
{
	int thid = 0;

	thid = sceKernelCreateThread("update_thread", CallbackThread, 0x11, 0xFA0, 0, 0);
	if(thid >= 0)
	{
		sceKernelStartThread(thid, 0, 0);
	}

	return thid;
}


//FROM SHPERICAL HARMONICS by adresd
// converts the image and uploads it to vram in one go
#define VRAM_OFFSET ((512*280*4)*3)
// 0x198000
static unsigned int vramaddr = 0;
unsigned char *convertimage(unsigned char *inptr,int size)
{
  // convert our raw image
  // saved as raw. no header .. interleaved and order RGB
  int x;
  unsigned char *input = inptr;
  unsigned char *output,*outptr;
  int tsize = size*size;
  if (vramaddr == 0)
    vramaddr = (0x40000000 | 0x04000000) + VRAM_OFFSET;
  outptr = output = (unsigned char *)vramaddr;
  for (x=0;x<tsize;x++) {
    *(outptr++) = *(input++);
    *(outptr++) = *(input++);
    *(outptr++) = *(input++);
    *(outptr++) = 0xff;
  }
  vramaddr += tsize * 4;
  if ((vramaddr & 0xff) != 0)
    vramaddr = (vramaddr & 0xffffff00) + 0x100;
  return output;
}


//#ifdef __cplusplus
//}
//#endif