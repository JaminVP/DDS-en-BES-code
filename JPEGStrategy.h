/**
 * @file JPEGStrategy.h
 * @ingroup DimensionalDataSource
 * @author Jamin Van Parys (jaminvp@gmail.com)
 * @date 19/03/2015
 * @brief Class which expands CompressionStrategy by defining the JPEG-compression for the received frames
 * @version 1.0
 */

#ifndef DimensionalDataSource_JPEGStrategy_h
#define DimensionalDataSource_JPEGStrategy_h
extern "C"{
#include <jpeglib.h>
}
#include "CompressionStrategy.h"

////////////////////
/*/JPEG FUNCTIONS/*/
////////////////////

/* setup the buffer */
void init_buffer(jpeg_compress_struct* cinfo) {}

/* what to do when the buffer is full; this should almost never
 * happen since we allocated our buffer to be big to start with
 */
boolean empty_buffer(jpeg_compress_struct* cinfo) {
    return TRUE;
}

/* finalize the buffer and do any cleanup stuff */
void term_buffer(jpeg_compress_struct* cinfo) {}


const static JOCTET EOI_BUFFER[1] = { JPEG_EOI };
struct my_source_mgr {
	struct jpeg_source_mgr pub;
	const JOCTET *data;
	size_t       len;
};

static void my_init_source(j_decompress_ptr cinfo) {}

static boolean my_fill_input_buffer(j_decompress_ptr cinfo) {
	my_source_mgr* src = (my_source_mgr*)cinfo->src;
	// No more data.  Probably an incomplete image;  just output EOI.
	src->pub.next_input_byte = EOI_BUFFER;
	src->pub.bytes_in_buffer = 1;
	return TRUE;
}
static void my_skip_input_data(j_decompress_ptr cinfo, long num_bytes) {
	my_source_mgr* src = (my_source_mgr*)cinfo->src;
	if (src->pub.bytes_in_buffer < num_bytes) {
		// Skipping over all of remaining data;  output EOI.
		src->pub.next_input_byte = EOI_BUFFER;
		src->pub.bytes_in_buffer = 1;
	} else {
		// Skipping over only some of the remaining data.
		src->pub.next_input_byte += num_bytes;
		src->pub.bytes_in_buffer -= num_bytes;
	}
}
static void my_term_source(j_decompress_ptr cinfo) {}

static void my_set_source_mgr(j_decompress_ptr cinfo, const unsigned char* data, size_t len) {
	my_source_mgr* src;
	if (cinfo->src == 0) { // if this is first time;  allocate memory
		cinfo->src = (struct jpeg_source_mgr *)(*cinfo->mem->alloc_small)
			((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(my_source_mgr));
	}
	src = (my_source_mgr*) cinfo->src;
	src->pub.init_source = my_init_source;
	src->pub.fill_input_buffer = my_fill_input_buffer;
	src->pub.skip_input_data = my_skip_input_data;
	src->pub.resync_to_restart = jpeg_resync_to_restart; // default
	src->pub.term_source = my_term_source;
	// fill the buffers
	src->data = (const JOCTET *)data;
	src->len = len;
	src->pub.bytes_in_buffer = len;
	src->pub.next_input_byte = src->data;
}


/**
 *
 * @brief Class that extends the abstract class CompressionStrategy. Offers the functionality to compress a depth matrix to a 12-bit JPEG-image
 */
class JPEGStrategy:public CompressionStrategy{
public:

    /**
     *
     * @brief Public method that offers the functionality to compress a depth matrix to a 12-bit JPEG-image
     * @param[in] xResolution The horizontal resolution of the used camera
     * @param[in] yResolution The vertical resolution of the used camera
     * @param[in] depthData A pointer to a depthmatrix containing the depth data
     * @param[out] out_buffer A pointer to a databuffer containing the compressed image
     * @param[out] nBytes The size of the out_buffer in bytes     */
    void compress_depth_to_image(const int& xResolution,const int& yResolution,short* depthData,unsigned char*& out_buffer_,int& nBytes){
		//std::cout<<"Xresolution: "<<xResolution<<" Yresolution: "<<yResolution<<" depthData: "<<depthData<<std::endl;
        //FILE* output = fopen(determinePathExtension(frame,"jpg").c_str(), "wb");
        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr       jerr;
        struct jpeg_destination_mgr dmgr;
        
        //jpeg_stdio_dest(&cinfo, output);
        
        //jpeg_memory_dest(&cinfo,jpgbuff,numBytes);
        /* create our in-memory output buffer to hold the jpeg */
        JOCTET* out_buffer   = new JOCTET[xResolution*yResolution*2];
        jpeg_create_compress(&cinfo);
        /* here is the magic */
        dmgr.init_destination    = init_buffer;
        dmgr.empty_output_buffer = empty_buffer;
        dmgr.term_destination    = term_buffer;
        dmgr.next_output_byte    = out_buffer;
        dmgr.free_in_buffer      = xResolution * yResolution *2;
        
        cinfo.image_width      = xResolution;
        cinfo.image_height     = yResolution;
        cinfo.input_components = 1;
        cinfo.in_color_space   = JCS_GRAYSCALE;
        
        cinfo.err = jpeg_std_error(&jerr);
        
        /* make sure we tell it about our manager */
        cinfo.dest = &dmgr;
        jpeg_set_defaults(&cinfo);
        /*set the quality [0..100]  */
        jpeg_set_quality (&cinfo, 75, true);
        jpeg_start_compress(&cinfo, true);
        int16_t *data=new int16_t[xResolution*yResolution];
		int16_t *data2=data;
        for(int i=0;i<xResolution*yResolution;i++){
            data[i]=depthData[i]/3;
        }
        
        JSAMPROW row_pointer;          /* pointer to a single row */
        while (cinfo.next_scanline < cinfo.image_height) {
            
            row_pointer = (JSAMPROW) data;
            jpeg_write_scanlines(&cinfo, &row_pointer, 1);
            data+=xResolution;
        }
        
        jpeg_finish_compress(&cinfo);
        out_buffer_=(unsigned char*)out_buffer;
        nBytes=cinfo.dest->next_output_byte - out_buffer;
        
        jpeg_destroy_compress(&cinfo);
		data=data2;
		delete[] data;
		
    }

	/**
	*
	* @brief Public method that offers the functionality to decompress a 12-bit JPEG-image to a depth matrix 
	* @param[in] xResolution The horizontal resolution of the used camera
	* @param[in] yResolution The vertical resolution of the used camera
	* @param[in] depthData A pointer to a depthmatrix containing the depth data
	* @param[out] out_buffer A pointer to a databuffer containing the compressed image
	* @param[out] nBytes The size of the out_buffer in bytes     */
	void decompress_image_to_depth(const int& xResolution,const int & yResolution,short*& data,unsigned char* image,int& nBytes){
		data=new short[xResolution*yResolution];
		struct jpeg_decompress_struct cinfo; 
		struct jpeg_error_mgr jerr;
		JSAMPARRAY buffer;        /* Output row buffer */
		buffer = (JSAMPARRAY)malloc(sizeof(JSAMPROW));
		int row_stride; 

		// Setup decompression structure
		cinfo.err = jpeg_std_error(&jerr); 
		jpeg_create_decompress(&cinfo); 
		my_set_source_mgr(&cinfo,image,xResolution*yResolution);

		// read info from header.
		int r = jpeg_read_header(&cinfo, TRUE);
		jpeg_start_decompress(&cinfo);
		row_stride = cinfo.output_width * cinfo.output_components;
		buffer[0]=(JSAMPROW)malloc(sizeof(JSAMPLE)*row_stride);
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

	}
};
#endif
