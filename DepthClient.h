/**
* @file DepthClient.h
* @ingroup BackendServer
* @author Jamin Van Parys (jaminvp@gmail.com)
* @date 09/04/2015
* @brief Class which represents a client who connected to the BackendServer
* @version 1.0
*/

#ifndef BackendServer_DepthClient_h
#define BackendServer_DepthClient_h

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <pcl/io/pcd_io.h>
#include "JPEGStrategy.h"
#include "PNGStrategy.h"
extern "C"{
#include <jpeglib.h>
}

using namespace std;
using boost::asio::ip::tcp;


/**
*
* @brief Class which represents a client who connected to the BackendServer
*/
class DepthClient
{

public:
	/**
	*
	* @brief Public constructor which starts initializes the DepthClient
	* @param[in] io_service The io_service provided by boost::asio
	* @param[in] id_ The id of the DepthClient
	* @param[in] visualizeClouds Indicates if the received pointclouds should be visualized
	*/
	DepthClient(boost::asio::io_service& io_service,const int& id_,bool visualizeClouds_=true)
		: socket_(io_service),frame(1),client_id(id_),kinfu_status(0),finished(false)
	{
		visualizeClouds=visualizeClouds_;
		viewer=NULL;
	}

	~DepthClient(){
		delete[] header;
		delete[] data_;
		delete decomp;
		if(viewer!=NULL) delete viewer;
	}

	/**
	*
	* @brief Public getter which returns the socket linked to the DepthClient
	*/
	tcp::socket& socket()
	{
		return socket_;
	}

	/**
	*
	* @brief Public method which starts the correspondance between the connected client and the server
	*/
	void start()
	{
		cout<<"Getting info from DepthClient..."<<endl;
		header= new int[3];
		//resolution (2 ints), compression method(1 int)
		socket_.async_read_some(boost::asio::buffer(header,3*sizeof(int)
			),
			boost::bind(&DepthClient::handle_first_read, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
	}

	/**
	*
	* @brief Public method which sends the completed KinFuWorld pointcloud back to the client
	* @param[in] worldbuffer The world pointcloud in a binary buffer
	*/
	void sendWorld(char*& worldbuffer, const int& size)
	{
		cout<<"Sending world back to client with ipv4-address: "<<socket_.remote_endpoint().address().to_string()<<" and size "<<size/1024<<"KB"<<endl;
		int* header2=new int[1];
		header2[0]=htonl(size);
		worldBuffer=worldbuffer;
		worldSize=size;
		boost::asio::async_write(socket_,boost::asio::buffer(header2,sizeof(int)),
			boost::bind(&DepthClient::handle_write,this,boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
		//resolution (2 ints), compression method(1 int)

	}



	string get_pointcloud_path(const int& i){
		return pointclouds[i];
	}

	int get_current_frame(){
		return pointclouds.size();
	}

	bool is_finished(){
		return finished;
	}

	int get_kinfu_status(){
		return kinfu_status;
	}

	void set_kinfu_status(int status){
		kinfu_status=status;
	}

private:

	/**
	*
	* @brief Private method used to determine a filepath for saving individual frames as a certain format to a local directory
	* @param[in] frame The frame number
	* @param[in] ext The extension of the file
	* @param[in] client_id The id of the owner of the frames
	*/
	string determinePathExtension(int frame,const int& client_id,const string &ext){
		stringstream filename;
		stringstream pathname;
		pathname<<"Release/pointclouds/"<<client_id;
		if(!boost::filesystem::is_directory(pathname.str())){
			boost::filesystem::create_directory(pathname.str());
		}

		filename<<"/frame";
		for(int i=10;i<MAXFILES;i*=10){
			if(frame%MAXFILES<i)filename<<"0";
		}
		filename<<(frame)%MAXFILES<<".";
		string filepath=pathname.str()+filename.str()+ext;
		return filepath;
	}

	/**
	*
	* @brief Private callback method used to handle the first header received by the server
	* @param[in] error A possible error code, is 0 if no error occured
	* @param[in] bytes_transferred The bytes that were transferred
	*/
	void handle_first_read(const boost::system::error_code& error,
		size_t bytes_transferred)
	{
		if (!error)
		{
			int s=3*sizeof(int);
			cout<<"Header bytes: "<<s<<endl;
			cout<<header<<endl;
			width=ntohl(header[0]);
			height=ntohl(header[1]);
			cout<<"Resolution: "<<width<<" x "<<height<<endl;
			compression=ntohl(header[2]);

			cout<<"Compression Strategy: "<<compression<<endl;
			if(compression==0){
				decomp= new JPEGStrategy();
				cout<<"JPEG decompression (12-bit)"<<endl;
			}
			else if(compression==1){
				decomp=new PNGStrategy();
				cout<<"PNG decompression"<<endl;
			}
			else{
				decomp=new JPEGStrategy();
				cout<<"Default decompression used: JPEG"<<endl;
			}

			boost::asio::async_read(socket_,boost::asio::buffer(header,sizeof(int)),
				boost::bind(&DepthClient::handle_read, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));

		}
		else
		{
			delete this;
		}
	}


	/**
	*
	* @brief Private callback method used to handle the header preceding the actual frames received by the server
	* @param[in] error A possible error code, is 0 if no error occured
	* @param[in] bytes_transferred The bytes that were transferred
	*/
	void handle_read(const boost::system::error_code& error,
		size_t bytes_transferred)
	{
		if (!error)
		{
			cout<<"-------------------------------------------------------------------"<<endl;
			cout<<"DepthClient "<<client_id<<": New frame, now reading header!"<<endl;
			received_int=ntohl(header[0]);
			cout<<"Number of bytes to receive: "<<received_int<<endl;
			data_=(unsigned char*)malloc(received_int);
			if(received_int!=0){
				boost::asio::async_read(socket_,boost::asio::buffer(data_,received_int),
					boost::bind(&DepthClient::handle_data, this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));
			}
			else{
				cout<<"DepthClient has exited. Total frames sent:"<<frame-1<<endl;
				if(viewer!=NULL) delete viewer;
				finished=true;
			}

		}
		else
		{
			delete this;
		}
	}

	/**
	*
	* @brief Private method used to convert a depth matrix with a given resolution to a pointcloud
	* @param[in] depthData The depth matrix.
	* @param[in] width Horizontal resolution.
	* @param[in] height Vertical Resolution.
	* @param[in] showPointcloud Determines if the cloud should be visualized
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr
		convertToXYZPointCloud (const int16_t depthData[], const int& width,const int& height,const int& compression=0) 
	{	

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
		//TODO: Variable resolution!
		cloud->height = height;
		cloud->width = width;
		cloud->is_dense = false;

		cloud->points.resize (cloud->height * cloud->width);
		register float constant_x = 1.0f/391;
		register float constant_y = 1.0f/463 ;
		register float centerX = ((float)cloud->width - 1.f) / 2.f;
		register float centerY = ((float)cloud->height - 1.f) / 2.f;

		float bad_point = std::numeric_limits<float>::quiet_NaN ();

		register int depth_idx = 0;
		for (int v = 0; v < cloud->height; ++v)
		{
			for (register int u = 0; u < cloud->width; ++u, ++depth_idx)
			{
				pcl::PointXYZ& pt = cloud->points[depth_idx];
				// Check for invalid measurements
				if (depthData[depth_idx] == 0)
				{
					// not valid
					pt.x = pt.y = pt.z = bad_point;
					continue;
				}
				if(compression==0)pt.z = depthData[depth_idx]*0.001f *3;
				else pt.z = depthData[depth_idx]*0.001f;
				pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
				pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
			}
		}
		cloud->sensor_origin_.setZero ();
		cloud->sensor_orientation_.w () = 0.0f;
		cloud->sensor_orientation_.x () = 1.0f;
		cloud->sensor_orientation_.y () = 0.0f;
		cloud->sensor_orientation_.z () = 0.0f;
		return (cloud);
	}

	/**
	*
	* @brief Private method used to handle the compressed image which was sent by the DDSClient
	* @param[in] error The possible error code that can be generated
	* @param[in] bytes_transferred The number of bytes that have been transferred
	*/
	void handle_data(const boost::system::error_code& error,
		size_t bytes_transferred)
	{
		if (!error)
		{

			cout<<"Transferred: "<<bytes_transferred<<endl;
			short* data;
			int b= bytes_transferred;
			decomp->decompress_image_to_depth(width,height,data,data_,b);
			string filepath=determinePathExtension(frame,client_id,"pcd");
			cout<<"Compression:"<<compression<<endl;
			cloud=convertToXYZPointCloud(data,width,height,compression);

			if(visualizeClouds && viewer==NULL){
				stringstream ss;
				ss<<"PointCloud Viewer DepthClient "<<client_id;
				cout<<ss.str()<<endl;
				viewer= new pcl::visualization::CloudViewer(ss.str());
				viewer->showCloud (cloud);
			}

			else if(visualizeClouds){
				cout<<viewer<<endl;
				viewer->showCloud (cloud);
			}
			cout<<"Saving as file: "<<filepath<<endl;
			pcl::io::savePCDFileBinaryCompressed(filepath,*cloud);
			pointclouds.push_back(filepath);
			delete data;
			frame++;
			cout<<" Done with frame!"<<endl;
			cout<<"-------------------------------------------------------------------"<<endl;
			boost::asio::async_read(socket_,boost::asio::buffer(header,sizeof(int)),
				boost::bind(&DepthClient::handle_read, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
		}
		else
		{
			cout<<"Transferred: "<<bytes_transferred<<endl;
			cout<<error<<endl;
			delete this;
		}
	}



	/**
	*
	* @brief Private callback method used to reply to the DDSClient who started the connection
	*/
	void handle_write(const boost::system::error_code& error,size_t bytes_transferred)
	{
		if (!error)
		{
			cout<<"Sending world to DepthClient with id "<<client_id<<"!"<<endl;
			socket_.get_io_service().reset();
			boost::asio::async_write(socket_,boost::asio::buffer(worldBuffer,worldSize),
				boost::bind(&DepthClient::finish_send_world, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
			socket_.get_io_service().run();
		}
		else
		{
			delete this;
		}
	}

	/**
	*
	* @brief Private method used to finish sending the finished KinfuWorld to the client
	* @param[in] error The possible error code that can be generated
	* @param[in] bytes_transferred The number of bytes that have been transferred
	*/
	void finish_send_world(const boost::system::error_code& error,
		size_t bytes_transferred)
	{
		if (!error)
		{ 
			cout<<"Finished sending the world for client "<<client_id<<endl;
			delete[] worldBuffer;
		}
		else
		{
			cout<<"Transferred: "<<bytes_transferred<<endl;
			cout<<error<<endl;
			delete this;
		}
	}

	tcp::socket socket_;
	enum { max_length = 307200 };
	unsigned char* data_;
	int received_int;
	int width;
	int height;
	int* header;
	int compression;
	int frame;
	int client_id;
	vector<string> pointclouds;
	int kinfu_status;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::visualization::CloudViewer* viewer;
	char* worldBuffer;
	bool finished;
	static const int MAXFILES=10000;
	CompressionStrategy* decomp;
	bool visualizeClouds;
	int worldSize;
};

#endif