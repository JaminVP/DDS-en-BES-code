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
#include <pcl/common/time.h>
#include "/Applications/opencv-2.4.8/3rdparty/libpng/png.h"
#include "JPEGClient.h"
#include "PNGClient.h"
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
const string DIRECTORY="/Users/Jamin/Kinect/depths/";
std::clock_t start;
StopWatch watch;
double duration;
JPEGClient* client;
std::vector<JOCTET> my_buffer;

///////////////////
/*/PNG FUNCTIONS/*/
///////////////////

/* structure to store PNG image bytes */
struct mem_encode
{
    char *buffer;
    size_t size;
};

void
my_png_write_data(png_structp png_ptr, png_bytep data, png_size_t length)
{
    /* with libpng15 next line causes pointer deference error; use libpng12 */
    struct mem_encode* p=(struct mem_encode*)png_get_io_ptr(png_ptr); /* was png_ptr->io_ptr */
    size_t nsize = p->size + length;
    
    /* allocate or grow buffer */
    if(p->buffer)
        p->buffer = (char*)realloc(p->buffer, nsize);
    else
        p->buffer = (char*)malloc(nsize);
    
    if(!p->buffer)
        png_error(png_ptr, "Write Error");
    
    /* copy new bytes to end of buffer */
    memcpy(p->buffer + p->size, data, length);
    p->size += length;
}

/* This is optional but included to show how png_set_write_fn() is called */
void
my_png_flush(png_structp png_ptr)
{
}

////////////////////
/*/JPEG FUNCTIONS/*/
////////////////////

/* setup the buffer but we did that in the main function */
void init_buffer(jpeg_compress_struct* cinfo) {}

/* what to do when the buffer is full; this should almost never
 * happen since we allocated our buffer to be big to start with
 */
boolean empty_buffer(jpeg_compress_struct* cinfo) {
	return TRUE;
}

/* finalize the buffer and do any cleanup stuff */
void term_buffer(jpeg_compress_struct* cinfo) {}

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
        client = new JPEGClient(640,480,222);
		interface = new pcl::OpenNIGrabber();
		boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_DepthJPEG, this, _1);

		interface->registerCallback (f);

		interface->start ();
		watch.reset();
		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}

		interface->stop ();

	}
    
    void runDepthPNG ()
	{
		cout<<"Writing "<<MAXFILES-1<<" frames to directory: "<<DIRECTORY<<endl;
        
		interface = new pcl::OpenNIGrabber();
		boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_DepthPNG, this, _1);
        
		interface->registerCallback (f);
        
		interface->start ();
		watch.reset();
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
		//cout<<"Frame: "<<frame;
		const xn::DepthMetaData &metadata=depth.get()->getDepthMetaData();
		int xResolution; 
		int yResolution; 

		// calculate the depth metadata 

		xResolution = metadata.XRes(); 
		yResolution = metadata.YRes();

		// get the depth data 
		const XnDepthPixel* depthData = metadata.Data(); 

		//FILE* output = fopen(determinePathExtension(frame,"jpg").c_str(), "wb");
		struct jpeg_compress_struct cinfo;
		struct jpeg_error_mgr       jerr;
        struct jpeg_destination_mgr dmgr;


		
		//jpeg_stdio_dest(&cinfo, output);

        //jpeg_memory_dest(&cinfo,jpgbuff,numBytes);
        /* create our in-memory output buffer to hold the jpeg */
        JOCTET * out_buffer   = new JOCTET[xResolution*yResolution*2];
        jpeg_create_compress(&cinfo);
        /* here is the magic */
        dmgr.init_destination    = init_buffer;
        dmgr.empty_output_buffer = empty_buffer;
        dmgr.term_destination    = term_buffer;
        dmgr.next_output_byte    = out_buffer;
        dmgr.free_in_buffer      = xResolution * yResolution *2;
        
		cinfo.image_width      = xResolution;
		cinfo.image_height     = yResolution;
		cinfo.input_components = 1;
		cinfo.in_color_space   = JCS_GRAYSCALE;
        
        cinfo.err = jpeg_std_error(&jerr);
		
        /* make sure we tell it about our manager */
        cinfo.dest = &dmgr;
		jpeg_set_defaults(&cinfo);
		/*set the quality [0..100]  */
		jpeg_set_quality (&cinfo, 75, true);
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

        int nBytes=cinfo.dest->next_output_byte - out_buffer;
        cout<<"Frame:"<<frame<<endl;
        client->sendToServer((unsigned char*)out_buffer,nBytes);

		jpeg_destroy_compress(&cinfo);

	}
    

    void saveDepthToPNG(string fileName, const boost::shared_ptr<openni_wrapper::DepthImage>& depth)
	{
		//cout<<"Frame: "<<frame;
		const xn::DepthMetaData &metadata=depth.get()->getDepthMetaData();
		int xResolution;
		int yResolution;
        
		// calculate the depth metadata
        
		xResolution = metadata.XRes();
		yResolution = metadata.YRes();
        
		// get the depth data
		const XnDepthPixel* depthData = metadata.Data();
        png_structp png_ptr;
        png_infop info_ptr;
        string str=determinePathExtension(frame,".png");
        const char* title=str.c_str();
        int strlen=str.length();
        // Open file for writing (binary mode)
        FILE* fp = fopen(title, "wb");
        if (fp == NULL) {
            fprintf(stderr, "Could not open file %s for writing\n", "test.png");
            exit(1);
        }
        /* static */
        struct mem_encode state;
        
        /* initialise - put this before png_write_png() call */
        state.buffer = NULL;
        state.size = 0;
        
        // Initialize write structure
        png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        if (png_ptr == NULL) {
            fprintf(stderr, "Could not allocate write struct\n");
            exit(1);
        }
        
        /* if my_png_flush() is not needed, change the arg to NULL */
        png_set_write_fn(png_ptr, &state, my_png_write_data, my_png_flush);
        
        // Initialize info structure
        info_ptr = png_create_info_struct(png_ptr);
        if (info_ptr == NULL) {
            fprintf(stderr, "Could not allocate info struct\n");
            exit(1);
        }
        
        //png_init_io(png_ptr, fp);
        
        // Write header (16bit grayscale)
        png_set_IHDR(png_ptr, info_ptr, xResolution, yResolution,
                     16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                     PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
        
        
        // Set title
        if (title != NULL) {
            char * a=new char[strlen+1];
            strcpy(a,title);
            png_text title_text;
            title_text.compression = PNG_TEXT_COMPRESSION_NONE;
            title_text.key = "Title";
            title_text.text = a;
            png_set_text(png_ptr, info_ptr, &title_text, 1);
        }
        png_write_info(png_ptr, info_ptr);
        
        
        // Write image data

        int y;
        for (y=0 ; y<yResolution ; y++) {
                png_bytep row_pointer =(png_bytep)depthData;
                png_write_row(png_ptr, row_pointer);
                depthData+=xResolution;
        }
        
        // End write
        png_write_end(png_ptr, NULL);
        //boost::shared_ptr<char>
        //fwrite(state.buffer, state.size,1 ,fp);
        //PNGClient client(frame,state.buffer,state.size);
        //client.runClient();
        if (fp != NULL) fclose(fp);
        if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        //if(state.buffer)delete state.buffer;
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
				duration = watch.getTime();
				std::cout<<"Written "<<frame-1<<"frames in "<< duration <<" seconds: "<<frame*1.0/duration<<" FPS"<<endl;
                
			}
		}
	}

	void cloud_cb_DepthJPEG (const boost::shared_ptr<openni_wrapper::DepthImage>& depth)
	{
		if (!viewer.wasStopped()){
			//viewer.showCloud (cloud);
            
			if(frame%FRAMERATE==0 && frame<MAXFILES){
				saveDepthToJPG(determinePath(frame),depth);
			}
			if(frame%100==0)cout<<frame<<" frames done!"<<endl;
			frame++;
			if(frame==MAXFILES){
                delete client;
				duration = watch.getTime()/1000.0;
				std::cout<<"Written "<<frame-1<<"frames in "<< duration <<" seconds: "<<frame*1.0/duration<<" FPS"<<endl;
				interface->stop();
			}
		}
	}
    
    void cloud_cb_DepthPNG (const boost::shared_ptr<openni_wrapper::DepthImage>& depth)
	{
		if (!viewer.wasStopped()){
			//viewer.showCloud (cloud);
            
			if(frame%FRAMERATE==0 && frame<=MAXFILES-1){
				saveDepthToPNG(determinePath(frame),depth);
			}
			if(frame%100==0)cout<<frame<<" frames done!"<<endl;
			frame++;
			if(frame==MAXFILES){
				duration = watch.getTime()/1000.0;
				std::cout<<"Written "<<frame-1<<"frames in "<< duration <<" seconds: "<<frame*1.0/duration<<" FPS"<<endl;
				interface->stop();
			}
		}
	}
private:
	int frame;
    pcl::Grabber* interface;
    pcl::visualization::PCLVisualizer viewer;
    JPEGClient* client;
};


