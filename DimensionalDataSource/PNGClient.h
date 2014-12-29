//
//  PNGClient.h
//  DimensionalDataSource
//
//  Created by Jamin Van Parys on 19/03/14.
//
//

#ifndef DimensionalDataSource_PNGClient_h
#define DimensionalDataSource_PNGClient_h


//
// blocking_tcp_echo_client.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2012 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
extern "C"{
#include <jpeglib.h>
}
#define HANDLE(object,ptrToMember)  (PNGClient::handleWrite)
using boost::asio::ip::tcp;




    
class PNGClient{
    public:
    
    
    PNGClient(const int &frame_,char* buf,const int& width_,const int& height_,const int& size_){
        frame=frame_;
        width=width_;
        height=height_;
        b=buf;
        size=size_;
    }
    
    void doNothing(const boost::system::error_code& error,std::size_t bytes_transferred ){
        //?
    }
    
    void handle(const boost::system::error_code& error,std::size_t bytes_transferred ){
        
        //Should handle the socket being closed properly and the data in the buffer being released
        if(!error){
            delete b;
            s->close();
            delete s;
            cout<<"Bytes sent: "<<bytes_transferred<<endl;
        }
        else{
            cout<<"Error:" <<error.message()<<endl;
        }
    }

    
    void runClient()
    {
        try
        {
            boost::asio::io_service io_service;
            tcp::resolver resolver(io_service);
            tcp::resolver::query query(tcp::v4(), host, port);
            tcp::resolver::iterator iterator = resolver.resolve(query);
            s=new boost::asio::ip::tcp::socket(io_service);
            boost::asio::connect(*s, iterator);
            header=new int[3];

            std::cout << "Sending jpg: frame"<<frame<<" Size: "<<size<<"Bytes ... "<<endl;
            
            int number_to_send = size; // Put your value
            int converted_number = htonl(number_to_send);
            header[0]=converted_number;
            
            number_to_send=width;
            converted_number=htonl(number_to_send);
            header[1]=converted_number;
            
            number_to_send=height;
            converted_number=htonl(number_to_send);
            header[2]=converted_number;
            
            boost::asio::async_write(*s,boost::asio::buffer(header, 3* sizeof(int)),boost::bind(&PNGClient::doNothing,this,boost::asio::placeholders::
                                                                                                                     error,
                                                                                                                     boost::asio::placeholders::bytes_transferred));

            boost::asio::async_write(*s, boost::asio::buffer(b, size),
                                     boost::bind(&PNGClient::handle,
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
        string port="1112";
        string host="157.193.215.48";
        char * b;
        int width;
        int height;
        int size;
        int frame;
        int* header;

        boost::asio::ip::tcp::socket* s;
    
};
    

#endif
