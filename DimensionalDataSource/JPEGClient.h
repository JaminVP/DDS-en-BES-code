#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <vector>
extern "C"{
#include <jpeglib.h>
}
#define HANDLE(object,ptrToMember)  (PNGClient::handleWrite)
using boost::asio::ip::tcp;
using namespace boost;
using namespace std;


struct Frame{
    unsigned char* buf;
    int size;
};
    
class
JPEGClient{
    public:
    
    
    JPEGClient(const int& width_,const int& height_,const int& location_){
		width=width_;
		height=height_;
		location=location_;
		port="1112";
		host="157.193.215.47";
		frame=1;
		
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(tcp::v4(), host, port);
        tcp::resolver::iterator iterator = resolver.resolve(query);
        s=new boost::asio::ip::tcp::socket(io_service);
        boost::asio::connect(*s, iterator);
		header= new int[3];

		int number_to_send = width; 
        int converted_number = htonl(number_to_send);
		header[0]=converted_number;

		number_to_send = height; 
        converted_number = htonl(number_to_send);
		header[1]=converted_number;

		number_to_send = location; 
        converted_number = htonl(number_to_send);
		header[2]=converted_number;

		std::cout << "Connecting with server at "<<host<<" Port: "<<port<<endl;
		std::cout<<"Frames have a resolution of: "<<width<<" x "<<height<<endl;
		boost::asio::async_write(*s,boost::asio::buffer(header, 3*sizeof(int)),boost::bind(&JPEGClient::doNothing,this,boost::asio::placeholders::
                                                                                                           error,
                                                                                                                     boost::asio::placeholders::bytes_transferred));

		io_service.run();
    }

	~JPEGClient(){
		std::cout << "Closing connection with server... "<<endl;;
        std::cout << "Checking connection stack"<<endl;
        while(!st.empty()){
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }
		int number_to_send =0; 
        int converted_number = htonl(number_to_send);
		header[0]=converted_number;
        io_service.reset();
		boost::asio::async_write(*s,boost::asio::buffer(header, sizeof(int)),boost::bind(&JPEGClient::finishSend,this,boost::asio::placeholders::
                                                                                                                     error,
                                                                                                            boost::asio::placeholders::bytes_transferred));
        io_service.run();

	}
    
    
    void finishSend(const boost::system::error_code& error,std::size_t bytes_transferred ){
        if(!error){
            cout<<"Connection with server has terminated!"<<endl;
            
        }
        else{
            cout<<"Error: "<<error<<endl;
        }
    }

    
    void doNothing(const boost::system::error_code& error,std::size_t bytes_transferred ){
        if(!error){
            cout<<"Connection with server has been made!"<<endl;
        }
        else{
            cout<<"Error: "<<error<<endl;
        }
    }
    
    void sendData(const boost::system::error_code& error,std::size_t bytes_transferred ){
        
        cout<<"Sending data"<<endl;
        io_service.reset();
        
        boost::asio::async_write(*s, boost::asio::buffer(b, size),
                                 boost::bind(&JPEGClient::handle,
                                             this,
                                             boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));
    }
    
    void handle(const boost::system::error_code& error,std::size_t bytes_transferred ){
        
        //Should handle the socket being closed properly and the data in the buffer being released
        if(!error){
            cout<<"Sending succesful: deleting data and popping stack"<<endl;
            delete[] b;
            st.pop();

            cout<<"Bytes sent: "<<bytes_transferred<<endl;
        }
        else{
            cout<<"Error:" <<error.message()<<endl;
        }
    }

    
    void sendToServer(unsigned char* buf,const int& size_)
    {
        try
        {
            st.push(0);
            cout<<"Sending frame "<<frame<<" with size: "<<size_<<"...";
			frame++;
			b=buf;
			size=size_;
            
			
            
            int number_to_send = size; // Put your value
            int converted_number = htonl(number_to_send);
			header2 = new int[1];
			header2[0]=converted_number;

			io_service.reset();
			boost::asio::async_write(*s, boost::asio::buffer(header2, sizeof(int)),
                                     boost::bind(&JPEGClient::sendData,
                                                  this,
                                                  boost::asio::placeholders::error,
                                                  boost::asio::placeholders::bytes_transferred));
			

            io_service.run();
            
            std::cout << "Done!"<<endl;
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << "\n";
        }
    }
    
    private:
        enum { max_length = 1024 };
        string port;
        string host;
        stack<int> st;
        
        unsigned char * b;
		int size;
		int width;
		int height;
		int frame;
		int location;
		int* header;
		int* header2;
		boost::asio::io_service io_service;
		boost::asio::ip::tcp::socket* s;
    
};
