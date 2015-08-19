/**
* @file DDSClient.h
* @ingroup DimensionalDataSource
* @author Jamin Van Parys (jaminvp@gmail.com)
* @date 16/03/2015
* @brief Handles the sending of compressed images to a backend server
* @version 1.0
*/
#ifndef __DDSCLIENT_H  
#define __DDSCLIENT_H

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <vector>
#include <queue>
#include "CompressionStrategy.h"
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include "ConfigReader.h"
using boost::asio::ip::tcp;
using namespace boost;
using namespace std;

int MAX_QUEUE=1000;
ConfigReader cfg("config.txt");
/**
*
* @brief Class that represents a compressed frame
*/
struct Frame{
public:
	Frame(unsigned char*& buf_,const int& size_){
		buf=buf_;
		size=size_;
	}
	unsigned char* buf;
	int size;
};

/**
*
* @brief Class that acts as a queue with a limited size for Frames
*/
class LimitedQueue{
public:
	LimitedQueue(int max=MAX_QUEUE){
		MAX_FRAMES=max;
		halvedRemaining=0;
	}

	void push(Frame& f){
		if(q.size()<=MAX_FRAMES){
			q.push(f);
		}
	}

	void reduceQueue(){
		cout<<"Attempting to reduce the queue..."<<endl;
		cout<<"Current halved frames remaining: "<<halvedRemaining<<endl;
		int size=q.size();
		for(int i=0;i<halvedRemaining;i++){
			Frame j=q.front();
			q.pop();
			q.push(j);
		}
		cout<<"Halving rest of the queue with remaining size: "<<q.size()-halvedRemaining<<"..."<<endl;
		bool skip=false;
		for(int i=0;i<size-halvedRemaining;i++){
			if(skip){
				Frame j=q.front();
				q.pop();
				delete[] j.buf;
			}
			else{
				Frame j=q.front();
				q.pop();
				push(j);
			}
			skip=!skip;
		}
		halvedRemaining=(MAX_FRAMES-halvedRemaining)/2 + halvedRemaining;
		cout<<"New halved frames remaining: "<<halvedRemaining<<endl;
		cout<<"Queue size: "<<q.size()<<endl;
		cout<<"Test: "<<q.front().buf<<endl;
	}

	Frame front(){
		return q.front();

	}

	void pop(){
		q.pop();
		if(halvedRemaining>0){
			halvedRemaining--;
		}
	}

	int size(){
		return q.size();
	}

	int remainingHalved(){
		return halvedRemaining;
	}

private:
	int halvedRemaining;
	queue<Frame> q;
	int MAX_FRAMES;
};

/**
*
* @brief Class that handles the sending of compressed images to a backend server
*/
class DDSClient{
public:

	/**
	*
	* @brief Constructor that establishes a connection with the backend server
	* @param[in] width_ The width of a single frame
	* @param[in] height_ The height of a single frame
	* @param[in] comp_ The used compression strategy
	*/
	DDSClient(const int& width_,const int& height_,const Compression& comp_){
		connection_open=true;
		receivedWorld=false;
		comp=comp_;
		width=width_;
		height=height_;
		
		port=cfg["port"];
		host=cfg["host"];
		frame=1;
		std::cout << "Connecting with server at "<<host<<" Port: "<<port<<"..."<<endl;
		try{
			tcp::resolver resolver(io_service);
			tcp::resolver::query query(tcp::v4(), host, port);
			tcp::resolver::iterator iterator = resolver.resolve(query);
			s=new boost::asio::ip::tcp::socket(io_service);
			boost::asio::connect(*s, iterator);
			if(header!=NULL)delete[] header;
			header= new int[3];

			int number_to_send = width;
			int converted_number = htonl(number_to_send);
			header[0]=converted_number;

			number_to_send = height;
			converted_number = htonl(number_to_send);
			header[1]=converted_number;
			if(comp==JPEG){

				cout<<"Using JPEG-Compression"<<endl;
				number_to_send=0;
			}
			else if(comp==PNG){
				number_to_send=1;
				cout<<"Using PNG-Compression"<<endl;
				//if more compression techniques are added, more cases need to be added
			}
			converted_number = htonl(number_to_send);
			header[2]=converted_number;


			std::cout<<"Frames have a resolution of: "<<width<<" x "<<height<<endl;
			boost::asio::async_write(*s,boost::asio::buffer(header, 3*sizeof(int)),boost::bind(&DDSClient::confirmConnection,this,boost::asio::placeholders::
				error,
				boost::asio::placeholders::bytes_transferred));

			io_service.run();

		}
		catch(...){

			cout<<"Couldn't connect to server! Press any key to exit the program."<<endl;
			cin.get();
			exit(0);
		}
	}

	/**
	*
	* @brief Destructor that closes the connection with the backend server if there's still a connection
	*/
	~DDSClient(){
		if(connection_open){
			closeConnection();
		}
	}

	/**
	*
	* @brief Closes the connection with the backend server
	*/
	void closeConnection(){

		std::cout << "Closing connection with server..."<<endl;
		std::cout << "Checking connection stack"<<endl;
		connection_open=false;
		mtx.lock();
		int size=remainingFrames.size();
		mtx.unlock();
		while(size>0){
			boost::this_thread::sleep (boost::posix_time::seconds (5));
			mtx.lock();
			size=remainingFrames.size();
			
			std::cout<<"Remaining frames: "<<size<<endl;
			mtx.unlock();
		}
		int number_to_send =0; 
		int converted_number = htonl(number_to_send);
		header[0]=converted_number;
		io_service.reset();
		boost::asio::async_write(*s,boost::asio::buffer(header, sizeof(int)),boost::bind(&DDSClient::finishTerminate,this,boost::asio::placeholders::
			error,
			boost::asio::placeholders::bytes_transferred));
		io_service.run();

	}


	/**
	*
	* @brief Public method. Puts a single frame on the queue of frames remaining to send.
	* @param[in] buf The buffer containing the compressed image
	* @param[in] size_ The size of the buffer (in bytes)
	*/
	bool sendFrameToServer(unsigned char* buf,const int& size_){
		//cout<<"New frame received with size: "<<size_<<endl;
		Frame f(buf,size_);
		
		mtx.lock();
		if(remainingFrames.size()==0){
			remainingFrames.push(f);
			mtx.unlock();
			boost::thread* thr = new boost::thread(boost::bind(&DDSClient::checkQueue, this));
		}
		else{
			if(remainingFrames.size()==MAX_QUEUE){
				if(remainingFrames.remainingHalved() ==MAX_QUEUE-1){
					cout<<"Error: queue for sending has reached its maximal size due to a low upload capacity. The current queue will be sent but no further frames will be allowed."<<endl;
					mtx.unlock();
					return false;
				}
				else{
					remainingFrames.reduceQueue();
					cout<<"Queue reduced, current size: "<<remainingFrames.size()<<endl;
				}
			}
			remainingFrames.push(f);
			mtx.unlock();
			return true;
		}

	}

	/**
	*
	* @brief Public method. Receives the size of the finished KinfuWorld from the Backend Server, asynchronous
	*/
	void receiveWorldFromServer(){
		cout<<"Waiting to receive the world from the backendserver..."<<endl;
		//first read the size of the world
		if(header3!=NULL)delete[] header;
		
		header3=new int[1];
		io_service.reset();
		s->async_read_some(boost::asio::buffer(header3,sizeof(int)
			),
			boost::bind(&DDSClient::handle_receive_world,this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
		io_service.run();
	}

	/**
	*
	* @brief Public method. Gives a string which represents the IPv4-address of the backendserver
	*/
	string getHost(){
		return host;
	}

	/**
	*
	* @brief Public method. Returns a string which represents the used port of the backendserver
	*/
	string getPort(){
		return port;
	}

	/**
	*
	* @brief Public method. Sets the IPv4-address of the backend server
	*/
	void setHost(const string& host_ ){
		host=host_;
	}

	/**
	*
	* @brief Public method. Sets the IPv4-address of the backend server
	*/
	void setPort(const string& port_){
		port=port_;
	}

private:
	/**
	*
	* @brief Private method. Sends a single compressed image to the backendserver
	* @param[in] buf The buffer containing the compressed image
	* @param[in] size_ The size of the buffer (in bytes)
	*/
	void sendToServer(unsigned char* buf,const int& size_)
	{
		try
		{

			frame++;
			b=buf;
			size=size_;

			if(frame%30==0){
				cout<<"Sending frame "<<frame<<" with size: "<<size_<<"..."<<endl;
				cout<<"Remaning frames in queue: "<<remainingFrames.size()<<endl;
			}

			int number_to_send = size; // Put your value
			int converted_number = htonl(number_to_send);
			if(header2!=NULL)delete[] header2;
			header2 = new int[1];
			header2[0]=converted_number;

			io_service.reset();
			boost::asio::async_write(*s, boost::asio::buffer(header2, sizeof(int)),
				boost::bind(&DDSClient::sendData,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
			io_service.run();

		}
		catch (std::exception& e)
		{
			std::cerr << "Exception: " << e.what() << "\n";
		}
	}

	/**
	*
	* @brief Private method which checks if the queue has a frame to send
	*/
	void checkQueue(){
		mtx.lock();
		//cout<<"The queue is being checked (size: "<<remainingFrames.size()<<")"<<endl;

		if(remainingFrames.size()==1){
			Frame f=remainingFrames.front();
			mtx.unlock();
			sendToServer(f.buf,f.size);
		}
		else{
			cout<<"This shouldn't happen"<<endl;
			mtx.unlock();
		}

	}

	/**
	*
	* @brief Private callback method used by the destructor, signals the connection being fully terminated or why it failed to do so
	*/
	void finishTerminate(const boost::system::error_code& error,std::size_t bytes_transferred ){
		if(!error){
			cout<<"Connection with server has terminated!"<<endl;
			receiveWorldFromServer();
			cout<<"Waiting";
			while(!receivedWorld){
				cout<<".";
				boost::this_thread::sleep (boost::posix_time::seconds (5));
			}
			cout<<" Done!"<<endl;
		}
		else{
			cout<<"Error finish: "<<error<<endl;
		}
	}


	/**
	*
	* @brief Private callback method used by the constructor, signals the connection being made or why it failed to do so
	*/
	void confirmConnection(const boost::system::error_code& error,std::size_t bytes_transferred ){
		if(!error){
			cout<<"Connection with server has been made!"<<endl;
		}
		else{
			cout<<"Error confirm: "<<error<<endl;
		}
	}

	/**
	*
	* @brief Private callback method used by sendToServer, sends the actual data after the a header has been sent to the backend server
	*/
	void sendData(const boost::system::error_code& error,std::size_t bytes_transferred ){

		io_service.reset();

		boost::asio::async_write(*s, boost::asio::buffer(b, size),
			boost::bind(&DDSClient::handle,
			this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
	}

	/**
	*
	* @brief Private method which handles the socket being closed properly and the data in the buffer being released
	*/
	void handle(const boost::system::error_code& error,std::size_t bytes_transferred ){


		if(!error){
			//cout<<"Sending succesful: deleting data and checking queue"<<endl;
			if(b!=NULL){
				delete[] b;
			}
			mtx.lock();
			remainingFrames.pop();
			if(remainingFrames.size()>0){
				Frame f=remainingFrames.front();
				mtx.unlock();
				sendToServer(f.buf,f.size);
			}
			else{
				mtx.unlock();
			}

			//cout<<"Bytes sent: "<<bytes_transferred<<endl;
		}
		else{
			cout<<"Error handle:" <<error.message()<<endl;
		}
	}

	/**
	*
	* @brief Private method which handles the socket being closed properly and the data in the buffer being released
	*/
	void handle_receive_world(const boost::system::error_code& error,std::size_t bytes_transferred ){
		if(!error){
			int sizeOfWorld=ntohl(header3[0]);
			cout<<"Receiving the finished KinfuWorld with size "<<sizeOfWorld/1024<<"KB"<<endl;
			worldBuffer= new char[sizeOfWorld];
			io_service.reset();
			boost::asio::async_read(*s,boost::asio::buffer(worldBuffer,sizeOfWorld),boost::bind(&DDSClient::finish_receive_world,this,boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
			io_service.run();
		}
		else{
			cout<<"Error handle:" <<error.message()<<endl;
		}
	}

	/**
	*
	* @brief Private method which finishes receiving the world from the BackendServer and saves it in a directory
	*/
	void finish_receive_world(const boost::system::error_code& error,std::size_t bytes_transferred ){
		if(!error){
			cout<<"Transferred: "<<bytes_transferred<<endl;
			string directory=cfg["worldDirectory"];
			stringstream ss;
			using namespace boost::filesystem;
			using namespace boost::lambda;

			path pad(directory);
			directory_iterator begin(pad), end;
			int n = count_if(begin, end,
				[](const directory_entry & d) {
					return !is_directory(d.path());
			});
			ss<<directory<<"world"<<n+1<<".pcd";
			cout<<ss.str()<<endl;
			FILE* f=fopen(ss.str().c_str(),"wb");
			fwrite(worldBuffer,sizeof(char),bytes_transferred,f);
			fclose(f);
			receivedWorld=true;
			delete[] worldBuffer;
			cout<<"Received the entire world!"<<endl;
		}
		else{
			cout<<"Transferred: "<<bytes_transferred<<endl;
			cout<<error<<endl;
			delete this;
		}

	}

	bool connection_open;
	string port;
	string host;
	LimitedQueue remainingFrames;
	Compression comp;
	unsigned char * b;
	char* worldBuffer;
	int size;
	int width;
	int height;
	int frame;
	int* header;
	int* header2;
	int* header3;
	boost::asio::io_service io_service;
	boost::asio::ip::tcp::socket* s;
	boost::mutex mtx;
	bool receivedWorld;

};
#endif