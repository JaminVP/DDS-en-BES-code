#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <iostream>


class SimpleONIViewer{
public:
	SimpleONIViewer () : frame(0),viewer ("PCL ONI Viewer"){}
	int frame;
	void cloud_cb_ (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > &cloud)
	{
		if (!viewer.wasStopped()){
			viewer.showCloud (cloud);
			frame++;
			std::cout<<frame<<" "<<std::endl;
		}
	}

	void run ()
	{
		pcl::Grabber* interface = new pcl::ONIGrabber("Streams/test2.oni",false,true);
		boost::function<void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&)> f =
			boost::bind (&SimpleONIViewer::cloud_cb_, this, _1);
		std::cout<<"FPS:"<<interface->getFramesPerSecond()<<std::endl;
		interface->registerCallback (f);
		interface->start ();
		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep (boost::posix_time::seconds (1));
		}
		interface->stop (); 
	}

private:
	pcl::visualization::CloudViewer viewer;
};
