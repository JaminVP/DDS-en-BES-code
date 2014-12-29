#include <iostream>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <ctime>
extern "C"{
#include <jpeglib.h>
}
using namespace boost::filesystem;
using namespace std;

//construct pointclouds by loading JPEG files and converting them to pointclouds
class PNGDepthGrabber{
public:
	PNGDepthGrabber(){}
	typedef unsigned short ushort;
	void run(const string& pa){
		double duration1 = std::clock();
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
					int16_t* data=new int16_t[307200];
					JSAMPARRAY buffer;        /* Output row buffer */
					buffer = (JSAMPARRAY)malloc(sizeof(JSAMPROW));
					int row_stride; 
					buffer[0]=(JSAMPROW)malloc(sizeof(JSAMPLE)*row_stride);
					struct jpeg_decompress_struct cinfo;
					struct jpeg_error_mgr jerr;
					cinfo.err = jpeg_std_error(&jerr);
					/* physical row width in output buffer */
					//TODO: Other resolutions? put resolution as first 2 numbers in file!
					FILE *fp;
					for(directory_iterator dir(p);dir!=end;dir++){
						double st=clock();
						jpeg_create_decompress(&cinfo);
						
						files++;
						fp=fopen(dir->path().string().c_str(),"rb");
						jpeg_stdio_src(&cinfo, fp);
						cout<<"Converting File: "<<dir->path()<<endl;
						jpeg_read_header(&cinfo, TRUE);
						jpeg_start_decompress(&cinfo);
						row_stride = cinfo.output_width * cinfo.output_components;

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
						fclose(fp);
						//cout<<"Decompression: "<<std::clock()-st<<" ms"<<endl;
						//st=std::clock();
						pcl::PointCloud<pcl::PointXYZ>::Ptr pcd=convertToXYZPointCloud(data);
						//cout<<"Conversion: "<<std::clock()-st<<" ms"<<endl;
						//st=clock();
						pcl::io::savePCDFileBinaryCompressed(determinePCDPath(files),*pcd);
						//cout<<"Saving: "<<std::clock()-st<<" ms"<<endl;
						cout<<" Done!"<<endl;

					}
					// converted to a path by the
					// path stream inserter

					double duration2=std::clock();
					double time=(duration2-duration1)*1.0/1000;
					cout<<"Loaded "<<files<< " files in "<<time<<" seconds"<<"( "<<(files*1.0)/time<<" FPS )"<<endl;
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
		convertToXYZPointCloud (const int16_t depthData[]) const
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
				pt.z = depthData[depth_idx] * 0.001f*3;
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
		string filepath="pointclouds/"+filename.str();
		return filepath;
	}

private:


};