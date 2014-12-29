#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h> 
#include "kinfuLS_app.cpp"
#include "VoxelGridFilter.h"
extern "C"{
	#include <jpeglib.h>
}
using namespace std;

const int MAXFILES=1000;
const string DIRECTORY="depths/";

using namespace pcl::gpu::kinfuLS;
using boost::asio::ip::tcp;
int total_clients;



string determinePathExtension(int frame,const int& client_id,const string &ext){
	stringstream filename;
	stringstream pathname;
	pathname<<"pointclouds/"<<client_id;
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


// This is based on the FILE source manager in libjpeg

const static JOCTET EOI_BUFFER[1] = { JPEG_EOI };
struct my_source_mgr {
	struct jpeg_source_mgr pub;
	const JOCTET *data;
	size_t       len;
};

static void my_init_source(j_decompress_ptr cinfo) {}

static boolean my_fill_input_buffer(j_decompress_ptr cinfo) {
	my_source_mgr* src = (my_source_mgr*)cinfo->src;
	// No more data.  Probably an incomplete image;  just output EOI.
	src->pub.next_input_byte = EOI_BUFFER;
	src->pub.bytes_in_buffer = 1;
	return TRUE;
}
static void my_skip_input_data(j_decompress_ptr cinfo, long num_bytes) {
	my_source_mgr* src = (my_source_mgr*)cinfo->src;
	if (src->pub.bytes_in_buffer < num_bytes) {
		// Skipping over all of remaining data;  output EOI.
		src->pub.next_input_byte = EOI_BUFFER;
		src->pub.bytes_in_buffer = 1;
	} else {
		// Skipping over only some of the remaining data.
		src->pub.next_input_byte += num_bytes;
		src->pub.bytes_in_buffer -= num_bytes;
	}
}
static void my_term_source(j_decompress_ptr cinfo) {}

static void my_set_source_mgr(j_decompress_ptr cinfo, const unsigned char* data, size_t len) {
	my_source_mgr* src;
	if (cinfo->src == 0) { // if this is first time;  allocate memory
		cinfo->src = (struct jpeg_source_mgr *)(*cinfo->mem->alloc_small)
			((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(my_source_mgr));
	}
	src = (my_source_mgr*) cinfo->src;
	src->pub.init_source = my_init_source;
	src->pub.fill_input_buffer = my_fill_input_buffer;
	src->pub.skip_input_data = my_skip_input_data;
	src->pub.resync_to_restart = jpeg_resync_to_restart; // default
	src->pub.term_source = my_term_source;
	// fill the buffers
	src->data = (const JOCTET *)data;
	src->len = len;
	src->pub.bytes_in_buffer = len;
	src->pub.next_input_byte = src->data;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
		convertToXYZPointCloud (const int16_t depthData[], const int& width,const int& height) 
	{	

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
		
		cloud->height = height;
		cloud->width = width;
		cloud->is_dense = false;

		cloud->points.resize (cloud->height * cloud->width);
		register float constant_x = 1.0f/525;
		register float constant_y = 1.0f/525 ;
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
				pt.z = depthData[depth_idx]*0.001f *3;
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

class Client
{
public:
	Client(boost::asio::io_service& io_service,const int& id_)
		: socket_(io_service),frame(1),client_id(id_),kinfu_status(0),finished(false)
	{
	}

	tcp::socket& socket()
	{
		return socket_;
	}

	void start()
	{
		cout<<"Getting info from client..."<<endl;
		header= new int[3];
		//resolution (2 ints), geographical location (1 int? dunno)
		socket_.async_read_some(boost::asio::buffer(header,3*sizeof(int)
			),
			boost::bind(&Client::handle_first_read, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
	}

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
			location=ntohl(header[2]);
			
			cout<<"Location: "<<location<<endl;
			stringstream ss;
			ss<<client_id<<endl;
			boost::asio::async_read(socket_,boost::asio::buffer(header,sizeof(int)),
				boost::bind(&Client::handle_read, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));

		}
		else
		{
			delete this;
		}
	}

	void handle_read(const boost::system::error_code& error,
		size_t bytes_transferred)
	{
		if (!error)
		{
			cout<<"-------------------------------------------------------------------"<<endl;
			cout<<"Client "<<client_id<<": New frame, now reading header!"<<endl;
			int s=sizeof(int);
			cout<<"Header bytes: "<<s<<endl;
			cout<<header<<endl;
			received_int=ntohl(header[0]);
			cout<<received_int<<endl;
			cout<<"Number of bytes to receive: "<<received_int<<endl;
			data_=(unsigned char*)malloc(received_int);
			if(received_int!=0){
			boost::asio::async_read(socket_,boost::asio::buffer(data_,received_int),
				boost::bind(&Client::handle_data, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
			}
			else{
				cout<<"Client has exited. Total frames sent:"<<frame-1<<endl;
				finished=true;
			}

		}
		else
		{
			delete this;
		}
	}

	void handle_data(const boost::system::error_code& error,
		size_t bytes_transferred)
	{
		if (!error)
		{
			
			cout<<"Transferred: "<<bytes_transferred<<endl;
			int16_t* data=new int16_t[width*height];
			struct jpeg_decompress_struct cinfo; 
			struct jpeg_error_mgr jerr;
			JSAMPARRAY buffer;        /* Output row buffer */
			buffer = (JSAMPARRAY)malloc(sizeof(JSAMPROW));
			int row_stride; 

			// Setup decompression structure
			cinfo.err = jpeg_std_error(&jerr); 
			jpeg_create_decompress(&cinfo); 
			my_set_source_mgr(&cinfo, data_,received_int);

			// read info from header.
			int r = jpeg_read_header(&cinfo, TRUE);
			jpeg_start_decompress(&cinfo);
			row_stride = cinfo.output_width * cinfo.output_components;
		    buffer[0]=(JSAMPROW)malloc(sizeof(JSAMPLE)*row_stride);
			int counter=0;
			
			while (cinfo.output_scanline < cinfo.output_height) {
				/* jpeg_read_scanlines expects an array of pointers to scanlines.
				* Here the array is only one element long, but you could ask for
				* more than one scanline at a time if that's more convenient.
				*/
				

				(void) jpeg_read_scanlines(&cinfo, buffer, 1);
				
				memcpy(data+counter,buffer[0],row_stride*2);
				counter+=row_stride;
				/* Assume put_scanline_someplace wants a pointer and sample count. */
			}
			(void) jpeg_finish_decompress(&cinfo);
			jpeg_destroy_decompress(&cinfo);
			string filepath=determinePathExtension(frame,client_id,"pcd");
			cloud=convertToXYZPointCloud(data,width,height);
			cout<<"Saving as file: "<<filepath<<endl;
			pcl::io::savePCDFileBinaryCompressed(filepath,*cloud);
			pointclouds.push_back(filepath);
			delete data;
			frame++;
			cout<<" Done with frame!"<<endl;
			cout<<"-------------------------------------------------------------------"<<endl;
			boost::asio::async_read(socket_,boost::asio::buffer(header,sizeof(int)),
				boost::bind(&Client::handle_read, this,
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


	void handle_write(const boost::system::error_code& error)
	{
		if (!error)
		{
			cout<<"Replying to new client!"<<endl;
			socket_.async_read_some(boost::asio::buffer(data_, max_length),
				boost::bind(&Client::handle_read, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
		}
		else
		{
			delete this;
		}
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
	tcp::socket socket_;
	enum { max_length = 307200 };
	unsigned char* data_;
	int received_int;
	int width;
	int height;
	int* header;
	int location;
	int frame;
	int client_id;
	vector<string> pointclouds;
	int kinfu_status;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	bool finished;
};

class JPEGServer
{
public:
	JPEGServer(boost::asio::io_service& io_service, short port)
		: io_service_(io_service),
		acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
	{
		
		Client* new_session = new Client(io_service_,total_clients);
		total_clients++;
		clients.push_back(new_session);
		acceptor_.async_accept(new_session->socket(),
			boost::bind(&JPEGServer::handle_accept, this, new_session,
			boost::asio::placeholders::error));

	}

	~JPEGServer( )
	{
		
		Client* new_session = new Client(io_service_,total_clients);
		total_clients++;
		clients.push_back(new_session);
		acceptor_.async_accept(new_session->socket(),
			boost::bind(&JPEGServer::handle_accept, this, new_session,
			boost::asio::placeholders::error));

	}
	Client * get_client(const int& i){
		return clients[i];
	}

	int get_total_clients(){
		return clients.size();
	}

	void handle_accept(Client* new_session,
		const boost::system::error_code& error)
	{ 
		if (!error)
		{
			new_session->start();
			new_session = new Client(io_service_,total_clients);
			cout<<"A new client has connected! ID:"<<total_clients<<endl;
			total_clients++;
			clients.push_back(new_session);
			acceptor_.async_accept(new_session->socket(),
				boost::bind(&JPEGServer::handle_accept, this, new_session,
				boost::asio::placeholders::error));
		}
		else
		{
			delete new_session;
		}
	}

private:
	boost::asio::io_service& io_service_;
	tcp::acceptor acceptor_; 
	vector<Client*> clients;
};

void KinfuLSClient(const int& i,JPEGServer* s);

void background_work(JPEGServer* s){
	while(1){
			cout<<"Total number of clients: "<<s->get_total_clients()-1<<endl;
			cout<<"-------------------------------------------------------------"<<endl;
			size_t current_available;
			size_t total_available;
			cudaMemGetInfo(&current_available,&total_available);
			cout<<current_available<<"MB / "<<total_available<<" MB Available"<<endl;
			for(int i=0;i<s->get_total_clients()-1;i++){
				cout<<"Client "<<i+1<<":\n\t -Total frames:"<<s->get_client(i)->get_current_frame()<<
					"\n\t -Stream Status: "<<(s->get_client(i)->is_finished()?"Done":"Busy");
				cout<<"\n\t -Kinfu Status: ";
				switch(s->get_client(i)->get_kinfu_status()){
					case 0:
						cout<<"Not started"<<endl;
						break;
					case 1:
						cout<<"Busy"<<endl;
						break;
					case 2:
						cout<<"Done"<<endl;
						break;
				}
				if(s->get_client(i)->is_finished() && s->get_client(i)->get_kinfu_status()==0 && (total_available-current_available)/1000>1000)
				{
					cout<<"Starting Kinect Fusion for Client" <<i+1<<endl;
					s->get_client(i)->set_kinfu_status(1);
					KinfuLSClient(i,s);
					stringstream ss;
					ss<<"pointclouds\\"<<i+1<<"\\world.pcd";

					cudaMemGetInfo(&current_available,&total_available);

					cout<<current_available/1000<<"MB / "<<total_available/1000<<" MB Available"<<endl;
					
					
				}

			}
			cout<<"-------------------------------------------------------------"<<endl;
			boost::this_thread::sleep(boost::posix_time::seconds(10));
		}
}

void KinfuLSClient(const int& i,JPEGServer* s){
	boost::shared_ptr<pcl::Grabber> capture;
	float fps_pcd = 0.0;
	stringstream ss;
	boost::filesystem::path full_path( boost::filesystem::current_path() );
	std::cout << "Current path is : " << full_path << std::endl;
	ss<<"pointclouds\\"<<i+1;
	full_path/=ss.str();
	cout<<full_path<<endl;
	vector<string> pcd_files;
	pcd_files=getPcdFilesInDir(full_path.string());
	// Sort the read files by name
	sort (pcd_files.begin (), pcd_files.end ());
	capture.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> (pcd_files, fps_pcd, false));
	float volume_size = 4.0f;
	float shift_distance = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
	int snapshot_rate = pcl::device::kinfuLS::SNAPSHOT_RATE;
	KinFuLSApp app (*capture, volume_size, shift_distance, snapshot_rate,pcd_files.size(),i+1);
	app.pcd_source_   = true;
	pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
	try { app.startMainLoop (true); }  
	catch (const pcl::PCLException& /*e*/) { cout << "PCLException" << endl; }
	catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
	catch (const std::exception& /*e*/) { cout << "Exception" << endl; }
	std::cout << "Client "<<i+1<<" KinectFusion done!"<<endl;
	s->get_client(i)->set_kinfu_status(2);
	capture.reset();
	for(int i=0;i<pcd_files.size();i++){
			boost::filesystem::remove(pcd_files[i]);
	}
	ss.clear();


}




void runServer(int port)
{
	try
	{
		cout<<"Starting JPEGServer!"<<endl;
		boost::asio::io_service io_service;
		total_clients=1;
		
		
		cout<<"Now listening on port "<<port<<endl;
		cout<<"//////////////////////////////////////////////////"<<endl;
		JPEGServer* s=new JPEGServer(io_service, port);
		boost::thread t(background_work,s);
		io_service.run();

		

	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}
}


