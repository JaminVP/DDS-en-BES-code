#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>
#include <math.h>
#include <fstream>
#include <boost/thread.hpp>
#include <ctime>
extern "C"{
	#include <jpeglib.h>
}
#include <stdio.h>
using namespace std;
using namespace pcl;
using namespace boost; 

const int FRAMERATE=1;//how many frames till next save
const int MAXFILES=100;
const string DIRECTORY="depths/";
std::clock_t start;
double duration;
pcl::Grabber* interface;

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer () : frame(1),viewer ("PCL OpenNI Viewer"){}
	//spin 180° over Z
	const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >
		spin_cloud180(
		const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >& cloud){
			Eigen::Matrix4f transformationMatrix;
			transformationMatrix<<
				cos(M_PI),-sin(M_PI),0,0
				,sin(M_PI),cos(M_PI),0,0,
				0,0,1,0,
				0,0,0,1;
			pcl::PointCloud<pcl::PointXYZRGBA> cloud_out;
			transformPointCloud<pcl::PointXYZRGBA>(*(cloud.get()),cloud_out,transformationMatrix);
			return cloud_out.makeShared();
	}





	void savePCDCompressed(int frame_,const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > &cloud){
		// stringstream to store compressed point cloud
		std::stringstream compressedData;
		bool showStatistics = false;

		// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

		pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> PointCloudEncoder(compressionProfile, showStatistics);
		PointCloudEncoder.encodePointCloud (cloud, compressedData);
		//cout<<"Saving compressed frame to file "<<filepath<<", Size: "<<cloud.get()->size()<<" ";
		ofstream of;
		of.open(determinePath(frame_));
		of.write(compressedData.str().c_str(),compressedData.str().length());
		of.close();
		//cout<<"Done!"<<endl;
	}

	void savePCDCompressedXYZ(int frame_,const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > &cloud){
		// stringstream to store compressed point cloud
		std::stringstream compressedData;
		bool showStatistics = false;

		// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

		pcl::io::OctreePointCloudCompression<pcl::PointXYZ> PointCloudEncoder(compressionProfile, showStatistics);
		PointCloudEncoder.encodePointCloud (cloud, compressedData);
		//cout<<"Saving compressed frame to file "<<filepath<<", Size: "<<cloud.get()->size()<<" ";
		ofstream of;
		of.open(determinePath(frame_));
		of.write(compressedData.str().c_str(),compressedData.str().length());
		of.close();
		//cout<<"Done!"<<endl;
	}


	void savePCDBinary(int frame_,const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > &cloud){
		//cout<<"Saving frame to file "<<filepath<<", Size: "<<cloud.get()->size()<<" ";
		pcl::io::savePCDFileBinaryCompressed(determinePath(frame_),*cloud.get());
		//cout<<"Done!"<<endl;
	}

	void savePCDBinaryXYZ(int frame_,const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > &cloud){
		//cout<<"Saving frame to file "<<filepath<<", Size: "<<cloud.get()->size()<<" ";
		pcl::io::savePCDFileBinaryCompressed(determinePath(frame_),*cloud.get());
		//cout<<"Done!"<<endl;
	}

	void savePCDASCII (int frame_,const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > &cloud){
		//cout<<"Saving frame to file "<<filepath<<", Size: "<<cloud.get()->size()<<" ";
		pcl::io::savePCDFileASCII(determinePath(frame_),*cloud.get());
		//cout<<"Done!"<<endl;
	}

	void savePCDASCIIXYZ (int frame_,const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > &cloud){
		//cout<<"Saving frame to file "<<filepath<<", Size: "<<cloud.get()->size()<<" ";
		pcl::io::savePCDFileASCII(determinePath(frame_),*cloud.get());
		//cout<<"Done!"<<endl;
	}



	string determinePath(int frame){
		stringstream filename;
		filename<<"frame";
		for(int i=10;i<MAXFILES;i*=10){
			if((frame/FRAMERATE)%MAXFILES<i)filename<<"0";
		}
		filename<<(frame/FRAMERATE)%MAXFILES<<".txt";
		string filepath=DIRECTORY+filename.str();
		return filepath;
	}

	string determinePathExtension(int frame,const string &ext){
		stringstream filename;
		filename<<"frame";
		for(int i=10;i<MAXFILES;i*=10){
			if((frame/FRAMERATE)%MAXFILES<i)filename<<"0";
		}
		filename<<(frame/FRAMERATE)%MAXFILES<<".";
		string filepath=DIRECTORY+filename.str()+ext;
		return filepath;
	}

	void run ()
	{
		start = std::clock();
		interface = new pcl::OpenNIGrabber();
		boost::function<void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

		interface->registerCallback (f);

		interface->start ();

		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}

		interface->stop (); 
	}

	void runDepthBinary ()
	{
		cout<<"Writing "<<MAXFILES-1<<" frames to directory: "<<DIRECTORY<<endl;

		interface = new pcl::OpenNIGrabber();
		boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_DepthBinary, this, _1);

		interface->registerCallback (f);

		interface->start ();
		start = std::clock();
		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}

		interface->stop (); 


	}

	void runDepthJPEG ()
	{
		cout<<"Writing "<<MAXFILES-1<<" frames to directory: "<<DIRECTORY<<endl;

		interface = new pcl::OpenNIGrabber();
		boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_DepthJPEG, this, _1);

		interface->registerCallback (f);

		interface->start ();
		start = std::clock();
		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}

		interface->stop (); 


	}

	void runXYZ ()
	{
		start = std::clock();
		interface = new pcl::OpenNIGrabber();
		boost::function<void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_XYZ, this, _1);

		interface->registerCallback (f);

		interface->start ();

		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}

		interface->stop (); 


	}
	pcl::visualization::CloudViewer viewer;


	void cloud_cb_ (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > &cloud)
	{
		if (!viewer.wasStopped()){
			//viewer.showCloud (cloud);

			if(frame%FRAMERATE==0 && frame<=MAXFILES-1){
				savePCDBinary(frame,cloud);
			}

			frame++;
			if(frame==MAXFILES){
				duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				std::cout<<frame<<" over "<< duration <<" seconds: "<<frame*1.0/duration<<" FPS"<<endl;
			}

		}
	}

	void cloud_cb_XYZ (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > &cloud)
	{
		if (!viewer.wasStopped()){
			//viewer.showCloud (cloud);

			if(frame%FRAMERATE==0 && frame<=MAXFILES-1){
				savePCDASCIIXYZ(frame,cloud);
			}
			frame++;
			if(frame==MAXFILES){
				duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				std::cout<<"Written "<<frame-1<<"frames in "<< duration <<" seconds: "<<frame*1.0/duration<<" FPS"<<endl;
			}
		}
	}


	typedef unsigned short ushort;
	void saveDepthToLocation(string fileName, const boost::shared_ptr<openni_wrapper::DepthImage>& depth) 
	{ 
		//cout<<"Frame: "<<frame;
		const xn::DepthMetaData &metadata=depth.get()->getDepthMetaData();
		int xResolution; 
		int yResolution; 

		// calculate the depth metadata 

		xResolution = metadata.XRes(); 
		yResolution = metadata.YRes(); 
		int bytesPerPixel = metadata.BytesPerPixel(); 
		int totalPixels = metadata.XRes() * metadata.YRes(); 

		// get the depth data 
		const XnDepthPixel* depthData = metadata.Data(); 

		FILE* pFile;
		pFile = fopen(fileName.c_str(), "wb");
		for (int r = 0; r < xResolution; ++r) 
		{ 
			for (int c = 0; c < yResolution; ++c) 
			{ 
				int16_t input=(int16_t)*depthData;
				fwrite(&input,sizeof(int16_t),1,pFile); 
				depthData++;
			} 
		} 
		fclose(pFile);
		//cout<<"Done"<<endl;
	}

	void saveDepthToJPG(string fileName, const boost::shared_ptr<openni_wrapper::DepthImage>& depth) 
	{ 
		cout<<"Frame: "<<frame;
		const xn::DepthMetaData &metadata=depth.get()->getDepthMetaData();
		int xResolution; 
		int yResolution; 

		// calculate the depth metadata 

		xResolution = metadata.XRes(); 
		yResolution = metadata.YRes(); 
		int bytesPerPixel = metadata.BytesPerPixel(); 
		int totalPixels = metadata.XRes() * metadata.YRes(); 

		// get the depth data 
		const XnDepthPixel* depthData = metadata.Data(); 

		FILE* output = fopen(determinePathExtension(frame,"jpg").c_str(), "wb");
		struct jpeg_compress_struct cinfo;
		struct jpeg_error_mgr       jerr;


		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_compress(&cinfo);
		jpeg_stdio_dest(&cinfo, output);

		cinfo.image_width      = xResolution;
		cinfo.image_height     = yResolution;
		cinfo.input_components = 1;
		cinfo.in_color_space   = JCS_UNKNOWN;

		jpeg_set_defaults(&cinfo);
		/*set the quality [0..100]  */
		jpeg_set_quality (&cinfo, 100, true);
		jpeg_start_compress(&cinfo, true);
		int16_t *data=new int16_t[307200];
		for(int i=0;i<307200;i++){
			data[i]=(int16_t)(depthData[i]/3);
		}

		JSAMPROW row_pointer;          /* pointer to a single row */
		while (cinfo.next_scanline < cinfo.image_height) {

			row_pointer = (JSAMPROW) data;
		    jpeg_write_scanlines(&cinfo, &row_pointer, 1);
			data+=640;
		}
		jpeg_finish_compress(&cinfo);
		jpeg_destroy_compress(&cinfo);
		fclose(output);
		cout<<"Done"<<endl;
	}

	void cloud_cb_DepthBinary (const boost::shared_ptr<openni_wrapper::DepthImage>& depth)
	{
		if (!viewer.wasStopped()){
			//viewer.showCloud (cloud);

			if(frame%FRAMERATE==0 && frame<=MAXFILES-1){
				saveDepthToLocation(determinePath(frame),depth);
			}
			if(frame%100==0)cout<<frame<<" frames done!"<<endl;
			frame++;
			if(frame==MAXFILES){
				duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				std::cout<<"Written "<<frame-1<<"frames in "<< duration <<" seconds: "<<frame*1.0/duration<<" FPS"<<endl;
				viewer.~CloudViewer();
			}
		}
	}

	void cloud_cb_DepthJPEG (const boost::shared_ptr<openni_wrapper::DepthImage>& depth)
	{
		if (!viewer.wasStopped()){
			//viewer.showCloud (cloud);

			if(frame%FRAMERATE==0 && frame<=MAXFILES-1){
				saveDepthToJPG(determinePath(frame),depth);
			}
			if(frame%100==0)cout<<frame<<" frames done!"<<endl;
			frame++;
			if(frame==MAXFILES){
				duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
				std::cout<<"Written "<<frame-1<<"frames in "<< duration <<" seconds: "<<frame*1.0/duration<<" FPS"<<endl;
				interface->stop();
			}
		}
	}
private:
	int frame;
};