#include <iostream>

//#include "build\SimpleOpenNIViewer.h"
//#include "build\SimpleONIViewer.h"
//#include "build\BinaryDepthGrabber.h"
//#include "build\JPEGDepthGrabber.h"
#include "build\JPEGServer.h"
//#include "build\PNGDepthGrabber.h"
//#include "build\PNGServer.h"
//#include "build\PCDMerger.h"
#include "build\IterativeClosestPoint.h"
//#include "build\NormalDistributionsTransform.h"
#include "build\InitialAlignment.h"

using namespace std;
int main (int argc,char** argv)
{
	
	//SimpleOpenNIViewer v;
	//v.runDepthJPEG ();

	//SimpleONIViewer o;
	//o.run();

	//BinaryDepthGrabber b;
	//b.run("depths/");
	
	
	//JPEGDepthGrabber j;
	//j.run("depths/");
	
	//runServer(1112);
	//runPNGServer(1112);

	//merge();
	startRegisterPairs(argc,argv);
	//NDT();
	//VoxelGridFilter("world.pcd");
	//align("object_templates.txt","world.pcd");
	cin.get();
	return 0;
}