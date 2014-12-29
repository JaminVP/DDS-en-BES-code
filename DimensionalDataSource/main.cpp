#include <iostream>
#include "SimpleOpenNIViewer.h"
#include "BinaryDepthGrabber.h"
#include "SimpleONIViewer.h"
#include "JPEGDepthGrabber.h"

using namespace std;

int main(){
	SimpleOpenNIViewer v;
    v.runDepthJPEG();
    
    
    //JPEGClient::runClient();
    //cin.get();
	return 0;
}