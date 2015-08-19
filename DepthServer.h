/** @defgroup BackendServer
*
*
* @brief BackendServer connects with multiple Dimensional Data Source Clients and receives their compressed frames through a TCP-connection. These frames then get decompressed using a DecompressionStrategy and converted to Pointclouds.
* @version 1.0
*/

/**
* @file DepthServer.h
* @ingroup BackendServer
* @author Jamin Van Parys (jaminvp@gmail.com)
* @date 09/04/2015
* @brief Class which receives compressed frames through boost::asio
* @version 1.0
*/

#ifndef BackendServer_DepthServer_h
#define BackendServer_DepthServer_h

#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "cuda.h"
#include <cuda_runtime_api.h>
#include <pcl/io/pcd_grabber.h>
#include "DepthClient.h"
#include "KinfuProcessor.h"
extern "C"{
#include <jpeglib.h>
}
using namespace std;


/**

* @brief Class that serves as a receiver of depth data. The depth data is to be decompressed by a specified DecompressionStrategy, converted to a Pointcloud and then saved to disk to be processed later 
*/
class DepthServer{
public:
	/**
	*
	* @brief Public constructor which starts the server on a given port and allows clients to connect
	* @param[in] io_service The io_service provided by boost
	* @param[in] port The port the server has to run on
	* @param[in] visualizeClouds_ Indicated if the received pointclouds should be visualized
	*/
	DepthServer(boost::asio::io_service& io_service, short port,bool visualizeClouds_=false)
		: io_service_(io_service),
		acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
	{
		visualizeClouds=visualizeClouds_;
		total_clients=1;
		DepthClient* new_session = new DepthClient(io_service_,total_clients,visualizeClouds);
		clients.push_back(new_session);
		acceptor_.async_accept(new_session->socket(),
			boost::bind(&DepthServer::handle_accept, this, new_session,
			boost::asio::placeholders::error));

	}

	/**
	* @brief Public destructor which starts the server on a given port and allows clients to connect
	*/
	~DepthServer(){
		for(int i=0;i<clients.size();i++){
			delete clients[i];
		}

	}

	/**
	* @brief Public getter which returns a pointer to a certain DepthClient
	* @param[in] i The index of the DepthClient
	*/
	DepthClient * get_client(const int& i){
		return clients[i];
	}

	/**
	* @brief Public getter which returns the total number of clients who have connected to the server
	*/
	int get_total_clients(){
		return clients.size();
	}

	/**
	* @brief Private method which periodically checks the status of all the connected clients and starts KinectFusion using KinfuProcessing if an available client has been found 
	*/
	void background_work(){
		while(1){
			cout<<"Total number of clients: "<<get_total_clients()-1<<endl;
			cout<<"-------------------------------------------------------------"<<endl;
			size_t current_available;
			size_t total_available;
			cudaMemGetInfo(&current_available,&total_available);
			cout<<current_available<<"MB / "<<total_available<<" MB Available"<<endl;
			for(int i=0;i<get_total_clients()-1;i++){
				cout<<"Client "<<i+1<<":\n\t -Total frames:"<<get_client(i)->get_current_frame()<<
					"\n\t -Stream Status: "<<(get_client(i)->is_finished()?"Done":"Busy");
				cout<<"\n\t -Kinfu Status: ";
				switch(get_client(i)->get_kinfu_status()){
				case 0:
					cout<<"Not started"<<endl;
					break;
				case 1:
					cout<<"Busy"<<endl;
					break;
				case 2:
					cout<<"Done"<<endl;
					break;
				case 3:
					cout<<"Failed"<<endl;
					break;
				}
				if(get_client(i)->is_finished() && get_client(i)->get_kinfu_status()==0 && (total_available-current_available)/1000>1000)
				{
					cout<<"Starting Kinect Fusion for Client" <<i+1<<endl;
					mtx.lock();
					get_client(i)->set_kinfu_status(1);
					mtx.unlock();
					char* worldbuffer;
					int size;
					int status=kinfuprocessor.ProcessKinfuClient(i,worldbuffer,size);
					cout<<"World size: "<<size%1024<<" KB"<<endl;
					get_client(i)->sendWorld(worldbuffer,size);


					//int status=1;
					mtx.lock();
					get_client(i)->set_kinfu_status(status);
					mtx.unlock();
					cudaMemGetInfo(&current_available,&total_available);

					cout<<current_available/1000<<"MB / "<<total_available/1000<<" MB Available"<<endl;


				}

			}
			cout<<"-------------------------------------------------------------"<<endl;
			boost::this_thread::sleep(boost::posix_time::seconds(10));
		}
	}

private:
	/**
	* @brief Private callbackmethod which handles a new DDSClient connecting to the BackendServer
	*/
	void handle_accept(DepthClient* new_session,
		const boost::system::error_code& error)
	{ 
		if (!error)
		{
			total_clients++;
			new_session->start();
			new_session = new DepthClient(io_service_,total_clients,visualizeClouds);
			cout<<"A new client has connected! ID:"<<total_clients<<endl;

			clients.push_back(new_session);
			acceptor_.async_accept(new_session->socket(),
				boost::bind(&DepthServer::handle_accept, this, new_session,
				boost::asio::placeholders::error));
		}
		else
		{
			delete new_session;
		}
	}

protected:
	boost::asio::io_service& io_service_;
	tcp::acceptor acceptor_; 
	boost::mutex mtx;
	vector<DepthClient*> clients;
	int total_clients;
	bool visualizeClouds;
	KinfuProcessor kinfuprocessor;
};




#endif
