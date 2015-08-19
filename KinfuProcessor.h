/**
* @file KinfuProcessing.h
* @ingroup BackendServer
* @author Jamin Van Parys (jaminvp@gmail.com)
* @date 09/04/2015
* @brief Class which processes the saved .pcd files for every DepthClient and converts it into a world .pcd using KinectFusion Largescale
* @version 1.0
*/

#ifndef BackendServer_KinfuProcessing_h
#define BackendServer_KinfuProcessing_h

#include <cstdlib>
#include <iostream>

#include <pcl/io/pcd_io.h>

#include "cuda.h"
#include <cuda_runtime_api.h>
#include "kinfuLS_app.cpp"
#include <pcl/io/pcd_grabber.h>

using namespace std;



/**

* @brief Class that processes saved pointclouds and converts them into a world pointcloud 
*/
class KinfuProcessor{
public:
	int ProcessKinfuClient(const int& i,char*& worldbuffer,int& size){
		boost::shared_ptr<pcl::Grabber> capture;
		float fps_pcd =0;
		stringstream ss;
		boost::filesystem::path full_path( boost::filesystem::current_path() );
		std::cout << "Current path is : " << full_path << std::endl;
		ss<<"Release\\pointclouds\\"<<i+1;
		full_path/=ss.str();
		cout<<full_path<<endl;
		vector<string> pcd_files;
		try{
			pcd_files=getPcdFilesInDir(full_path.string());
		}
		catch (const std::exception& e) { 
			cout << "Exception:"<< e.what() << endl;
			return(3);
		}
		// Sort the read files by name
		std::cout << "Sorting the "<<pcd_files.size()<<" pointcloud files..." << std::endl;
		sort (pcd_files.begin (), pcd_files.end ());
		try {
			capture.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> (pcd_files, fps_pcd, false));
			float volume_size = 4.0f;
			float shift_distance = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
			int snapshot_rate = pcl::device::kinfuLS::SNAPSHOT_RATE;

			KinFuLSApp app (*capture, volume_size, shift_distance, snapshot_rate,pcd_files.size(),i+1);
			app.registration_ = true;
			app.pcd_source_   = true;
			pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
			app.startMainLoop (true); 
		}  
		catch (const pcl::PCLException& /*e*/) { cout << "PCLException" << endl; return 3;}
		catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; return 3;}
		catch (const std::exception& e) { cout << "Exception:"<< e.what() << endl; return 3;}
		std::cout << "Client "<<i+1<<" KinectFusion done!"<<endl;
		capture.reset();
		for(int i=0;i<pcd_files.size();i++){
			boost::filesystem::remove(pcd_files[i]);
		}
		//Filtering
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		// Fill in the cloud data
		pcl::PCDReader reader;
		ss<<"/world"<<i+1<<".pcd";
		cout<<"Filtering "<<ss.str()<<" with a statistical outlier method..."<<endl;
		reader.read<pcl::PointXYZ> (ss.str(), *cloud);

		std::cerr << "Cloud before filtering: " << std::endl;
		std::cerr << *cloud << std::endl;

		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*cloud_filtered);

		std::cerr << "Cloud after filtering with statistical outlier filter: " << std::endl;
		std::cerr << *cloud_filtered << std::endl;

		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setInputCloud(cloud_filtered);
		vg.setLeafSize(2.0,2.0,2.0);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
		vg.filter(*cloud_out);

		std::cerr << "Cloud after filtering with voxelgrid filter: " << std::endl;
		std::cerr << *cloud_out << std::endl;

		pcl::PCDWriter writer;

		writer.write<pcl::PointXYZ> (ss.str(), *cloud_out, false);

		
		std::ifstream t;
		int length;
		t.open(ss.str().c_str());
		t.seekg(0,std::ios::end);
		length=t.tellg();
		size=length;
		t.seekg(0,std::ios::beg);
		worldbuffer= new char[length];
		t.read(worldbuffer,length);
		t.close();
		

		ss.clear();
		return 2;

	}

};


#endif
