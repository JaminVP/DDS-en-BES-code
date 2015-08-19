#include <iostream>

#include "build\DepthServer.h"
#include "build\NormalDistribution.h"
using namespace std;

void runServer(int port,bool showPointClouds)
{
	try
	{
		cout<<"Starting BackendServer!"<<endl;
		boost::asio::io_service io_service;
		stringstream pathname;
		pathname<<boost::filesystem::current_path().string()<<"/"<<"Release/pointclouds/";

		cout<<"Clearing '"<<pathname.str()<<"'"<<endl;
		namespace fs=boost::filesystem;
		fs::path path_to_remove(pathname.str());
		for (fs::directory_iterator end_dir_it, it(path_to_remove); it!=end_dir_it; ++it) {
			remove_all(it->path());
		}
		cout<<"Now listening on port "<<port<<endl;
		cout<<"//////////////////////////////////////////////////"<<endl;
		DepthServer* s=new DepthServer(io_service, port,showPointClouds);
		boost::thread(&DepthServer::background_work,s);
		io_service.run();



	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}
}

int main (int argc,char** argv)
{

	runServer(1112,false);
	//NormalDistribution();
	cin.get();
	return 0;
}