/**
 * @file CompressionStrategy.h
 * @ingroup DimensionalDataSource
 * @author Jamin Van Parys (jaminvp@gmail.com)
 * @date 19/03/2015
 * @brief Abstract Class which defines the compression strategy for the received frames
 * @version 1.0
 */

#ifndef DimensionalDataSource_CompressionStrategy_h
#define DimensionalDataSource_CompressionStrategy_h

/**
 *
 * @brief Enumerator which represents the used compression strategy, can be expanded
 */
enum Compression {JPEG,PNG};

/**
 
 * @brief Abstract class which defines the used compression stategy and the implementation to convert a depth image to a compressed image
 */
class CompressionStrategy{
public:
    /**
     *
     * @brief Public method that offers the functionality to compress a depth matrix to a compressed image
     * @param[in] xResolution The horizontal resolution of the used camera
     * @param[in] yResolution The vertical resolution of the used camera
     * @param[in] depthData A pointer to a depthmatrix containing the depth data
     * @param[out] out_buffer A pointer to a databuffer containing the compressed image
     * @param[out] nBytes The size of the out_buffer in bytes     */
    virtual void compress_depth_to_image(const int& xResolution,const int& yResolution,short* depthData,unsigned char*& out_buffer_, int& nBytes)=0;
    virtual void decompress_image_to_depth(const int& xResolution,const int& yResolution,short*& depthData,unsigned char* image,int& nBytes)=0;
};
#endif
