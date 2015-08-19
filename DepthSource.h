/** @defgroup DimensionalDataSource
*
*
* @brief Dimensional Data Source provides a consistent stream of compressed mages and sends them to a backend server through a single TCP-connection using the Boost-libraries
* @version 1.0
*/

/**
* @file DepthSource.h
* @ingroup DimensionalDataSource
* @author Jamin Van Parys (jaminvp@gmail.com)
* @date 16/03/2015
* @brief Abstract class which provides a stream of compressed depthimages
* @version 1.0
*/

#ifndef DimensionalDataSource_DepthSource_h
#define DimensionalDataSource_DepthSource_h

#include <iostream>

#include <math.h>
#include <fstream>
#include <boost/thread.hpp>
#include "DDSClient.h"
#include "JPEGStrategy.h"
#include "PNGStrategy.h"



/**

* @brief Abstract class that serves as a source of depth data. The depth data is to be compressed by a specified CompressionStrategy and then sent to the backend server using a DDSClient */
class DepthSource{
public:
	/**
	*
	* @brief Public constructor which initializes the correct CompressionStrategy
	*/

	DepthSource(Compression comp=JPEG){
		if(comp==JPEG){
			cout<<"Compression: using JPEG"<<endl;
			compression= new JPEGStrategy();
		}
		else if(comp==PNG){
			cout<<"Compression: using PNG"<<endl;
			compression= new PNGStrategy();
		}
		else compression=new JPEGStrategy();

		xResolution_=0;
		yResolution_=0;
		DIRECTORY="/Users/Jamin/Kinect/depths/";
		fileCount=0;
	}

	virtual ~DepthSource(){
		delete compression;
	}

	/**

	* @brief Public virtual method which starts the Grabber-interface
	*/
	virtual void runDepthSource()=0;

	/**

	* @brief Public virtual method which stops the Grabber-interface
	*/
	virtual void stopDepthSource()=0;

	/**
	*
	* @brief Virtual method.Saves a specified number of frames as to be determined images in a directory
	* @param[in] directory The directory where the frames will be saved
	* @param[in] N The number of frames that have to be saved
	*/
	virtual void saveFramesToDirectory( const string& directory_,int N=MAXFILES)=0;
	/*
	* @brief Sets the server address and the port to send images to
	*/
	void setServerSettings(const string& address,const string& port){
		client->setHost(address);
		client->setPort(port);
	}


protected:

	CompressionStrategy* compression;
	DDSClient* client;
	int frame;


	int fileCount;
	const static int MAXFILES=5000;
	string DIRECTORY;
	int xResolution_;
	int yResolution_;


};




#endif
