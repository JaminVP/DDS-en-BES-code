#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <png.h>



using boost::asio::ip::tcp;
int PNGFrame;

 struct mem_encode{
	char* buffer;
	png_uint_32 size;
	png_uint_32 current_pos;

};

class PNGsession
{
public:
	PNGsession(boost::asio::io_service& io_service)
		: socket_(io_service)
	{
	}

	tcp::socket& socket()
	{
		return socket_;
	}

	void start()
	{
		received_int=0;
		socket_.async_read_some(boost::asio::buffer(&received_int,sizeof(received_int)),
			boost::bind(&PNGsession::handle_read, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
	}

	void handle_read(const boost::system::error_code& error,
		size_t bytes_transferred)
	{
		if (!error)
		{
			cout<<"New PNGFrame, now reading!"<<endl;
			cout<<"Number of bytes to receive: "<<ntohl(received_int)<<endl;
			boost::asio::socket_base::receive_buffer_size option(1000000);
			socket_.set_option(option);
			boost::asio::async_read(socket_,boost::asio::buffer(data_,ntohl(received_int)),
			boost::bind(&PNGsession::handle_data, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));

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
			cout<<"Saving as file: "<<determinePathExtension(PNGFrame,"png")<<endl;
			FILE* fp=fopen("test.png","wb");
			fwrite(data_,bytes_transferred,1,fp);
			cout<<(png_sig_cmp((png_const_bytep)data_,0,8)?"Not valid png":"Valid png")<<endl;

			fclose(fp);

			//get PNG file info struct (memory is allocated by libpng)
			png_structp png_ptr = NULL;
			png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
			if (!png_ptr) {
				
				 std::cerr << "ERROR: Couldn't initialize png read struct" << std::endl;
				 cin.get();
				return; //Do your own error recovery/handling here
			}
			// get PNG image data info struct (memory is allocated by libpng)
			png_infop info_ptr = NULL;
			info_ptr = png_create_info_struct(png_ptr);
			if (!info_ptr) {
			 std::cerr << "ERROR: Couldn't initialize png info struct" << std::endl;
			 cin.get();
			 png_destroy_read_struct(&png_ptr, (png_infopp)0, (png_infopp)0);
			 return; //Do your own error recovery/handling here
			}
			setjmp(png_jmpbuf(png_ptr));
			struct mem_encode pngdata;
			
			pngdata.buffer=data_;

			pngdata.size=(png_uint_32)bytes_transferred;
			pngdata.current_pos=0;
			png_set_sig_bytes(png_ptr, 8);
			png_set_read_fn(png_ptr,&pngdata, ReadDataFromBuffer);
			//Start reading the png header

			png_read_info(png_ptr,info_ptr);
			
			//png_uint_32 width = 0;
			//png_uint_32 height = 0;
			//int bitDepth = 0;
			//int colorType = -1;
			//png_uint_32 retval = png_get_IHDR(png_ptr, info_ptr,
			//	&width,
			//	&height,
			//	&bitDepth,
			//	&colorType,
			//	NULL, NULL, NULL);
			
			PNGFrame++;
			cout<<"hoi"<<endl;
			cout<<" Done!"<<endl;
		}
		else
		{
			cout<<error.message()<<" Bytes received: "<<bytes_transferred<<endl;
			delete this;
		}
	}

	static void ReadDataFromBuffer(png_structp png_ptr, png_bytep outBytes,
		png_size_t byteCountToRead){
			struct mem_encode* p=(struct mem_encode*)png_get_io_ptr(png_ptr);
			size_t nsize=p->size + byteCountToRead;
			
			/*allocate or grow buffer */
			if(byteCountToRead>(p->size-p->current_pos)) png_error(png_ptr,"read error in read_data_memory (loadpng)");

			/* copy new bytes */
			memcpy(outBytes,p->buffer + p->current_pos,byteCountToRead);
			p->current_pos+=byteCountToRead;
	}
	

	void handle_write(const boost::system::error_code& error)
	{
		if (!error)
		{
			cout<<"Replying to new client!"<<endl;
			socket_.async_read_some(boost::asio::buffer(data_, max_length),
				boost::bind(&PNGsession::handle_read, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
		}
		else
		{
			delete this;
		}
	}

private:
	tcp::socket socket_;
	enum { max_length = 307200 };
	char data_[max_length];
	int received_int;
};

class PNGServer
{
public:
	PNGServer(boost::asio::io_service& io_service, short port)
		: io_service_(io_service),
		acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
	{
		PNGFrame=1;
		PNGsession* new_session = new PNGsession(io_service_);
		
		acceptor_.async_accept(new_session->socket(),
			boost::bind(&PNGServer::handle_accept, this, new_session,
			boost::asio::placeholders::error));

	}

	void handle_accept(PNGsession* new_session,
		const boost::system::error_code& error)
	{
		if (!error)
		{
			new_session->start();
			new_session = new PNGsession(io_service_);
			
			acceptor_.async_accept(new_session->socket(),
				boost::bind(&PNGServer::handle_accept, this, new_session,
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
};

void runPNGServer(int port)
{
	try
	{
		cout<<"Starting PNGServer!"<<endl;
		boost::asio::io_service io_service;

		PNGServer s(io_service, port);
		cout<<"Now listening on port "<<port<<endl;
		io_service.run();

	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}
}


