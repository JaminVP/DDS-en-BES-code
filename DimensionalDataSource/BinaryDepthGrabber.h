#include <iostream>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <ctime>
using namespace boost::filesystem;
using namespace std;

double duration1;
double duration2;
//construct pointclouds by loading binary depth files and converting them to pointclouds
class BinaryDepthGrabber{
public:
	BinaryDepthGrabber(){}
	typedef unsigned short ushort;
	void run(const string& pa){
		duration1 = std::clock();
		path p (pa);  
		try
		{
			if (exists(p))    // does p actually exist?
			{
				if (is_regular_file(p))        // is p a regular file?   
					cout << p << " size is " << file_size(p) << '\n';

				else if (is_directory(p))      // is p a directory?
				{
					//cout << p << " is a directory containing:\n";
					int files=0;
					directory_iterator end;
					directory_iterator dir(p);
					//TODO: Other resolutions? put resolution as first 2 numbers in file!
					for(directory_iterator dir(p);dir!=end;dir++){
						FILE *fp;
						files++;
						fp=fopen(dir->path().string().c_str(),"rb");
						//cout<<"File: "<<dir->path()<<endl;
						int16_t bfr[307200];
						fread(&bfr, sizeof(int16_t),307200, fp);
						pcl::PointCloud<pcl::PointXYZ>::Ptr pcd=convertToXYZPointCloud(bfr,pa);
						pcl::io::savePCDFileBinaryCompressed(determinePCDPath(files),*pcd);
					}
					// converted to a path by the
					// path stream inserter

					duration2=std::clock();
					cout<<"Loaded "<<files<< " files ("<<files*600000<<" Bytes ) in "<<(duration2-duration1)*1.0/1000<<" seconds"<<endl;
				}

				else
					cout << p << " exists, but is neither a regular file nor a directory\n";
			}
			else
				cout << p << " does not exist\n";
		}

		catch (const filesystem_error& ex)
		{
			cout << ex.what() << '\n';
		}
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr
		convertToXYZPointCloud (const int16_t depthData[],const string& path) const
	{
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
		//TODO: Variable resolution!
		cloud->height = 480;
		cloud->width = 640;
		cloud->is_dense = false;

		cloud->points.resize (cloud->height * cloud->width);
		register float constant_x = 1.0f;
		register float constant_y = 1.0f ;
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
				pt.z = depthData[depth_idx] * 0.001f;
				pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
				pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
			}
		}
		cloud->sensor_origin_.setZero ();
		cloud->sensor_orientation_.w () = 1.0f;
		cloud->sensor_orientation_.x () = 0.0f;
		cloud->sensor_orientation_.y () = 0.0f;
		cloud->sensor_orientation_.z () = 0.0f;  
		return (cloud);
	}

	string determinePCDPath(int frame){
		stringstream filename;
		filename<<"frame";
		for(int i=10;i<MAXFILES;i*=10){
			if((frame/FRAMERATE)<i)filename<<"0";
		}
		filename<<(frame/FRAMERATE)%MAXFILES<<".pcd";
		string filepath=DIRECTORY+filename.str();
		return filepath;
	}

private:


};