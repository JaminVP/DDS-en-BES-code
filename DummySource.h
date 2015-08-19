/**
* @file DummySource.h
* @ingroup DimensionalDataSource
* @author Jamin Van Parys (jaminvp@gmail.com)
* @date 16/03/2015
* @brief Class which implements the DepthSource abstract class. Acts as a dummy source as it sends the frames located in a directory to the backendserver through a DDSClient
* @version 1.0
*/

#ifndef DimensionalDataSource_DummySource_h
#define DimensionalDataSource_DummySource_h

#include "DepthSource.h"
#include <boost\filesystem.hpp>
#include "ConfigReader.h"


namespace fs= ::boost::filesystem;

/**

* @brief Class that extends the abstract class DepthSource. Acts as a dummysource of depthimages by sending compressed images that have previously been recorded by another depthsource and have been saved locally in a directory
*/
class DummySource:public DepthSource{
public:
	/**
	*
	* @brief Public constructor which initializes a connection with the DDSClient and locates the directory needed
	* @param[in] directory The directory where the frames are located
	* @param[in] xResolution The horizontal resolution of the frames
	* @param[in] yResolution The vertical resolution of the frames
	* @param[in] comp The used CompressionStrategy 
	*/
	DummySource(const string& directory,int xResolution,int yResolution,Compression comp):DepthSource(comp){
		DIRECTORY=directory;
		cenum=comp;
		xResolution_=xResolution;
		yResolution_=yResolution;
		client=new DDSClient(xResolution_,yResolution_,comp);
	}

	/**
	*
	* @brief Public destructor which terminates the connection with the backend server

	*/
	~DummySource(){

	}
	/**
	*
	* @brief Public method which starts the dummysource and sends the frames located in the directory to the server
	*/
	void runDepthSource (){
		string ext;
		if(cenum == JPEG){
			ext=".jpg";
		}
		else if(cenum == PNG){
			ext=".png";
		}
		else{
			cout<<"Not supported, using JPEG"<<endl;
			ext=".jpg";
		}
		if(!fs::exists(DIRECTORY) || !fs::is_directory(DIRECTORY)){
			cout<<DIRECTORY<<" does not exist or is not a directory!"<<endl;
			exit(1);
		}
		fs::recursive_directory_iterator it(DIRECTORY);
		fs::recursive_directory_iterator endit;
		while(it != endit){
			if(fs::is_regular_file(*it) && it->path().extension()==ext){
				string p=it->path().string();
				FILE* file=fopen(p.c_str(),"rb");
				fseek(file,0,SEEK_END);
				long fsize =ftell(file);
				fseek(file,0,SEEK_SET);
				unsigned char* buf=new unsigned char[fsize+1];
				fread(buf,fsize,1,file);
				client->sendFrameToServer(buf,fsize);
				fclose(file);
				//cout<<it->path()<<endl;
				files.push_back(it->path());
			}
			++it;
		}
		delete client;


	}

	/**

	* @brief Public virtual method which stops the Grabber-interface
	*/
	virtual void stopDepthSource(){
		cout<<"Not supported"<<endl;
	};

	/**
	*
	* @brief Virtual method.Saves a specified number of frames as to be determined images in a directory
	* @param[in] directory The directory where the frames will be saved
	* @param[in] N The number of frames that have to be saved
	*/
	virtual void saveFramesToDirectory( const string& directory_,int N=MAXFILES){
		cout<<"Not supported"<<endl;
	};

private:
	Compression cenum;
	vector<fs::path> files;

};


#endif
