// DimensionalDataSourceK2.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include "KinectV2Source.h"
#include "DummySource.h"
#include "ConfigReader.h"
using namespace std;
int _tmain(int argc, _TCHAR* argv[])
{
	ConfigReader cfg("config.txt");
	cfg.showSettings();
	Compression c;
	cin.get();
	if(cfg["compression"]=="PNG"){
		c=PNG;
	}
	else if(cfg["compression"]=="JPEG"){
		c=JPEG;
	}
	else{
		cerr<<"Unvalid compression type!"<<endl;
		c=PNG;
		exit(1);
	}
	
	string workMode=cfg["workMode"];
	if(workMode=="offline"){
		KinectV2Source k(c);
		k.saveFramesToDirectory(cfg["offlineDirectory"],atoi(cfg["framesToSave"].c_str()));
	}
	else if(workMode=="streaming"){
		KinectV2Source k(c);
		k.runDepthSource();
	}
	else if(workMode=="sendSavedFrames"){
		DummySource k(cfg["offlineDirectory"],atoi(cfg["offlineResolutionX"].c_str()),atoi(cfg["offlineResolutionY"].c_str()),c);
		k.runDepthSource();
	}
	
	cin.get();
	return 0;
}

