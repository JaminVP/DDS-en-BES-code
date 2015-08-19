/**
* @file Kinectv2.h
* @author Jamin Van Parys (jaminvp@gmail.com)
* @ingroup DimensionalDataSource
* @date 19/03/2015
* @brief Class of DimensionalDataSource which uses a KinectV2-camera to receive frames and uses a CompressionStrategy to compress them
* @version 1.0
*/

#ifndef DimensionalDataSource_KinectV2Source_h
#define DimensionalDataSource_KinectV2Source_h

#include "DepthSource.h"
#include <opencv2\opencv.hpp>
#include <Kinect.h>


/**

* @brief Safe release for the Kinect2-interfaces
*/
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


/**

* @brief Class that extends the abstract class DepthSource. Links a Kinect2-interface to a private callback method. This method compresses the received frame with a specific CompressionStrategy and sends it to the backend server using a DDSClient
*/
class KinectV2Source: public DepthSource{

public:
	/**
	*
	* @brief Constructor that initializes the IKinectSensor-interface
	* @param[in] comp The used CompressionStrategy (enumerator)
	*/
	KinectV2Source(Compression comp):DepthSource(comp){
		frame=0;
		cenum=comp;
		HRESULT hResult=S_OK;
		hResult=GetDefaultKinectSensor(&kinectv2sensor);
		if(FAILED(hResult)){
			std::cerr<<"Error: GetDefaultKinectSensor"<<std::endl;
			exit(1);
		}

		hResult=kinectv2sensor->Open();
		if( FAILED( hResult ) ){
			std::cerr << "Error : IKinectSensor::Open()" << std::endl;
			exit(1);
		}

		hResult = kinectv2sensor->get_DepthFrameSource( &depthSource );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
			exit(1);
		}

		// Reader
		hResult = depthSource->OpenReader( &depthReader );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
			exit(1);
		}
		// Description
		hResult = depthSource->get_FrameDescription( &description );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
			exit(1);
		}

		description->get_Width( &xResolution_ ); // 512
		description->get_Height( &yResolution_ ); // 424

		bufferSize =xResolution_ * yResolution_ * sizeof( unsigned short );
		// Range ( Range of Depth is 500-8000[mm], Range of Detection is 500-4500[mm] )
		unsigned short min = 0;
		unsigned short max = 0;
		depthSource->get_DepthMinReliableDistance( &min ); // 500
		depthSource->get_DepthMaxReliableDistance( &max ); // 4500
		SafeRelease(description);
		std::cout<< "Connection with Kinect v2-camera established!"<<endl;
		cout<<"Resolution: "<<xResolution_<<" x "<<yResolution_<<endl;
		std::cout << "Range : " << min << " - " << max << std::endl;
		cout<<"Compression: "<<comp<<endl;
	}
	/**
	*
	* @brief Destructor that releases all the interfaces safely
	*/
	~KinectV2Source(){
		SafeRelease(description);
		SafeRelease( depthReader);
		SafeRelease( depthSource);
		SafeRelease( kinectv2sensor);
	}

	/**
	*
	* @brief Public method which starts the KinectV2-interface, which results in the camera filming frames
	*/
	void runDepthSource ()
	{  
		sendFrames=false;
		IDepthFrame* pDepthFrame = NULL;
		UINT nBufferSize = 0;
		UINT16 *pBuffer =new UINT16[xResolution_*yResolution_];

		cout<<endl;
		framesSent=0;
		cv::Mat bufferMat( yResolution_,xResolution_, CV_16UC1 );
		cv::Mat depthMat(yResolution_,xResolution_, CV_8UC1 );
		cv::namedWindow( "Depth" ,CV_WINDOW_AUTOSIZE);
		while(1){

			HRESULT hr =depthReader->AcquireLatestFrame(&pDepthFrame);

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);  
				hr = pDepthFrame->AccessUnderlyingBuffer( &nBufferSize, reinterpret_cast<UINT16**>( &bufferMat.data ) );
			}
			if (SUCCEEDED(hr))
			{


				frame++;
				bufferMat.convertTo( depthMat, CV_8U, -255.0f / 8000.0f, 255.0f );

				if(sendFrames){
					// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
					cv::Mat img_rgb(depthMat.size(), CV_8UC3);

					// convert grayscale to color image
					cv::cvtColor(depthMat, img_rgb, CV_GRAY2RGB);
					cv::rectangle(img_rgb,cv::Point(0,0),cv::Point(xResolution_,yResolution_),cv::Scalar( 0, 0, 255 ),5,8);
					cv::circle( img_rgb,
						cv::Point(20,20),
						10,
						cv::Scalar( 0, 0, 255 ),
						-1,
						8 );
					cv::imshow( "Depth", img_rgb );
					framesSent++;
					
					cloud_callback( pBuffer,nBufferSize);
				}
				else{
					cv::imshow( "Depth", depthMat );
				}


			}
			if( cv::waitKey( 10 ) == VK_ESCAPE ){
				exit(0);
			}
			if(cv::waitKey( 10 ) == VK_SPACE ){
				sendFrames=!sendFrames;
				cout<<"Streaming toggled: ";

				cout<<(sendFrames? "Enabled" : "Disabled");
				cout<<endl;
				if(sendFrames){

					client=new DDSClient(xResolution_,yResolution_,cenum);
					cout<<"Sending frames, press SPACE to stop sending"<<endl;
				}
				else{
					cout<<"Sending stopped, "<<framesSent<<" frames sent"<<endl;
					framesSent=0;
					boost::thread* t = new boost::thread(boost::bind(&DDSClient::closeConnection,client));
				}

			}
			SafeRelease(pDepthFrame);
		}
	}

	/**
	*
	* @brief Public method which stops the KinectV2-interface, making the camera stop filming
	*/
	void stopDepthSource ()
	{
		SafeRelease(description);
		SafeRelease( depthReader);
		SafeRelease( depthSource);
		SafeRelease( kinectv2sensor);
	}

	/**
	*
	* @brief Saves a specified number of frames as images in a directory. If the directory already contains frames with the same name they will be overwritten.
	* @param[in] directory The directory where the frames will be saved
	* @param[in] N The number of frames that have to be saved
	*/
	void saveFramesToDirectory(const string& directory_,int N=MAXFILES){
		DIRECTORY=directory_;
		if(N>MAXFILES){
			fileCount=MAXFILES;
		}
		else fileCount=N;
		bool saveFrames=false;
		IDepthFrame* pDepthFrame = NULL;
		UINT nBufferSize = 0;
		UINT16 *pBuffer =new UINT16[xResolution_*yResolution_];

		int framesSaved=0;
		cv::Mat bufferMat( yResolution_,xResolution_, CV_16UC1 );
		cv::Mat depthMat(yResolution_,xResolution_, CV_8UC1 );
		cv::namedWindow( "Depth" ,CV_WINDOW_AUTOSIZE);
		while(1){

			HRESULT hr =depthReader->AcquireLatestFrame(&pDepthFrame);

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);  
				hr = pDepthFrame->AccessUnderlyingBuffer( &nBufferSize, reinterpret_cast<UINT16**>( &bufferMat.data ) );
			}
			if (SUCCEEDED(hr))
			{
				frame++;
				bufferMat.convertTo( depthMat, CV_8U, -255.0f / 8000.0f, 255.0f );


				if(saveFrames){
					// create 8bit color image. IMPORTANT: initialize image otherwise it will result in 32F
					cv::Mat img_rgb(depthMat.size(), CV_8UC3);

					// convert grayscale to color image
					cv::cvtColor(depthMat, img_rgb, CV_GRAY2RGB);
					cv::rectangle(img_rgb,cv::Point(0,0),cv::Point(xResolution_,yResolution_),cv::Scalar( 255, 0, 0 ),5,8);
					stringstream ss;
					ss<<framesSaved+1<<"/"<<fileCount;
					cv::putText(img_rgb,ss.str(), cvPoint(30,30), 
						cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,0,0), 1, CV_AA);
					cv::imshow( "Depth", img_rgb );
					framesSaved++;
					save_callback( pBuffer,nBufferSize,framesSaved);
					if(framesSaved==fileCount){
						cout<<"Saving finished! "<<framesSaved<<" frames have been saved to directory '"<<DIRECTORY<<"'"<<endl;
						saveFrames=false;
						framesSaved=0;
					}
				}
				else{
					cv::imshow( "Depth", depthMat );
				}


			}
			if( cv::waitKey( 10 ) == VK_ESCAPE && !saveFrames){
				exit(0);
			}
			if(cv::waitKey( 10 ) == VK_SPACE && !saveFrames ){
				saveFrames=true;
				cout<<"Starting saving frames!"<<endl;

			}
			SafeRelease(pDepthFrame);
		}
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
	* @brief Private callback method used to handle the individual frames and send them to backendserver
	* @param[in] pBuffer The buffer cointaining the depth values (matrix)
	* @param[in] ext nBufferSize
	*/
	void cloud_callback( UINT16 *pBuffer,UINT nBufferSize )
	{
		unsigned char* out_buffer;
		int nBytes;
		compression->compress_depth_to_image(xResolution_,yResolution_,(short*)pBuffer,out_buffer,nBytes);
		//if(cenum==PNG){
			//short* depth;
			//compression->decompress_image_to_depth(xResolution_,yResolution_,depth,out_buffer,nBytes);
			//bool equal=true;
			//for(int i=0;i<xResolution_*yResolution_;i++){
			//	equal=depth[i]==pBuffer[i];
			//}
			//cout<<"Equal after decompression? "<<(equal?"Yes":"No")<<endl;

			//cv::Mat img(cv::Size(xResolution_,yResolution_), CV_16UC1,pBuffer );
			//cv::Mat copyimg;
			//cv::bitwise_not(img,copyimg);

			// Display in window and wait for key press
			//cv::namedWindow("test");
			//cv::imshow("test", copyimg);
			//cout<<endl;
		//}
		if(!client->sendFrameToServer(out_buffer,nBytes)){
					cout<<"Sending stopped, "<<framesSent<<" frames sent"<<endl;
					framesSent=0;
					sendFrames=!sendFrames;
					
					boost::thread* t = new boost::thread(boost::bind(&DDSClient::closeConnection,client));
		}
	}

	/**
	*
	* @brief Private callback method used to handle the individual frames and saves them in a directory
	*/
	void save_callback(UINT16 *pBuffer,UINT nBufferSize,int framesSaved)
	{
		unsigned char* out_buffer;
		int nBytes;
		compression->compress_depth_to_image(xResolution_,yResolution_,(short*)pBuffer,out_buffer,nBytes);
		FILE* output;
		if(cenum==JPEG){
			output = fopen(determinePathExtension(framesSaved,"jpg").c_str(),"wb");
		}
		else if(cenum==PNG){
			output = fopen(determinePathExtension(framesSaved,"png").c_str(),"wb");
		}
		else{
			output=fopen(determinePathExtension(framesSaved,"jpg").c_str(),"wb");
		}

		fwrite(out_buffer,nBytes,1,output);
		fclose(output);
		delete[] out_buffer;
	}

	Compression cenum;
	IKinectSensor* kinectv2sensor;
	IDepthFrameSource* depthSource;
	IDepthFrameReader* depthReader;
	IFrameDescription* description;

	unsigned int bufferSize;
	int framesSent;
	bool sendFrames;
};



#endif
