/**
 * @file OpenNISource.h
 * @author Jamin Van Parys (jaminvp@gmail.com)
 * @ingroup DimensionalDataSource
 * @date 16/03/2015
 * @brief Class of DimensionalDataSource which uses an OpenNI-camera to receive frames and uses a CompressionStrategy to compress them
 * @version 1.0
 */
#ifndef DimensionalDataSource_OpenNISource_h
#define DimensionalDataSource_OpenNISource_h
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/compression/compression_profiles.h>
#include "DepthSource.h"
#include <ctime>
#include <stdio.h>
#include "JPEGStrategy.h"


/**
 
 * @brief Class that extends the abstract class DepthSource. Links an OpenNI Grabber-interface to a private callback method. This method compresses the received frame with a specific CompressionStrategy and sends it to the backend server using a DDSClient
 */
class OpenNISource: public DepthSource{

public:
    /**
     *
     * @brief Constructor that links the Grabber-interface to a private callback method
     */
    OpenNISource(Compression comp):DepthSource(comp){
        interface = new pcl::OpenNIGrabber();
        boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> f =
        boost::bind (&OpenNISource::start_callback, this, _1);
        interface->registerCallback(f);
        interface->start ();
        while(xResolution_!=0){
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }
        client = new DDSClient(xResolution_,yResolution_,comp);
        cout<<"Connected with server!"<<endl;
    }
    
    /**
     *
     * @brief Public method which starts the OpenNI Grabber-interface, which results in the camera filming frames
     */
    void runDepthSource ()
    {
        boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> f =
        boost::bind (&OpenNISource::cloud_callback, this, _1);
        interface->registerCallback(f);
        interface->start ();

        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }
        
        interface->stop ();
    }
    
    /**
     *
     * @brief Public method which stops the Grabber-interface, making the camera stop filming
     */
    void stopDepthSource ()
    {
        viewer.close();
        delete client;
        interface->stop ();
    }
    
    /**
     *
     * @brief Saves a specified number of frames as images in a directory
     * @param[in] directory The directory where the frames will be saved
     * @param[in] N The number of frames that have to be saved
     */
    void saveFramesToDirectory(const string& directory_,int N=MAXFILES){
        DIRECTORY=directory_;
        if(N>MAXFILES){
            fileCount=N;
        }
        boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> f =
        boost::bind (&OpenNISource::save_callback, this, _1);
        interface->registerCallback(f);
    }
    
private:
    
    /**
     *
     * @brief Private method used to determine a filepath for saving individual frames to a local directory
     * @param[in] frame The frame number
     * @param[in] ext The extension of the file
     */
    string determinePathExtension(int frame,const string &ext){
        stringstream filename;
        filename<<"frame";
        for(int i=10;i<MAXFILES;i*=10){
            if((frame)%MAXFILES<i)filename<<"0";
        }
        filename<<(frame)%MAXFILES<<".";
        string filepath=DIRECTORY+filename.str()+ext;
        return filepath;
    }
    
    /**
     *
     * @brief Private callback method used to find the camera resolution    */
    void start_callback(const boost::shared_ptr<openni_wrapper::DepthImage>& depth)
    {
            if(frame==1){
                interface->stop();
                frame=0;
            }
            else{
                
                const xn::DepthMetaData &metadata=depth.get()->getDepthMetaData();
                
                // calculate the depth metadata
                
                xResolution_= metadata.XRes();
                yResolution_= metadata.YRes();
                frame++;
            }
    }
    
    
    /**
     *
     * @brief Private callback method used to handle the individual frames and send them to backendserver
     */
    void cloud_callback(const boost::shared_ptr<openni_wrapper::DepthImage>& depth)
    {
        if (!viewer.wasStopped()){
            //viewer.showCloud (cloud);
            unsigned char* out_buffer;//depth_image in memory
            int nBytes;
            const xn::DepthMetaData &metadata=depth.get()->getDepthMetaData();
            int xResolution;
            int yResolution;
            
            // calculate the depth metadata
            
            xResolution = metadata.XRes();
            yResolution = metadata.YRes();
            
            // get the depth data
            const XnDepthPixel* depthData = metadata.Data();
            compression->compress_depth_to_image(xResolution,yResolution,(short*) depthData,out_buffer ,nBytes);
            client->sendToServer(out_buffer,nBytes);
            if(frame%100==0)cout<<frame<<" frames sent!"<<endl;
            frame++;
        }
    }
    
    /**
     *
     * @brief Private callback method used to handle the individual frames and save them in a directory
     */
    void save_callback(const boost::shared_ptr<openni_wrapper::DepthImage>& depth)
    {
        if (!viewer.wasStopped()){
            //viewer.showCloud (cloud);
            unsigned char* out_buffer;//Jpeg in memory
            int nBytes;
            const xn::DepthMetaData &metadata=depth.get()->getDepthMetaData();
            int xResolution;
            int yResolution;
            
            // calculate the depth metadata
            
            xResolution = metadata.XRes();
            yResolution = metadata.YRes();
            
            // get the depth data
            const XnDepthPixel* depthData = metadata.Data();
            
            compression->compress_depth_to_image(xResolution,yResolution,(short*) depthData,out_buffer ,nBytes);
            FILE* output = fopen(determinePathExtension(frame,"jpg").c_str(), "wb");
            fwrite(out_buffer, nBytes,1 ,output);
            fclose(output);
            if(frame%10==0)cout<<frame<<" frames saved in "<<DIRECTORY<<"!"<<endl;
            frame++;
            if(frame==fileCount){
                delete client;
                interface->stop();
            }
        }
    }

    
    
    pcl::Grabber* interface;
    pcl::visualization::PCLVisualizer viewer;
    
};




#endif
