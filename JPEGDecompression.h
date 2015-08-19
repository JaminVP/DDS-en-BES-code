/**
* @file JPEGDecompression.h
* @ingroup DimensionalDataSource
* @author Jamin Van Parys (jaminvp@gmail.com)
* @date 16/04/2015
* @brief Class which expands DecompressionStrategy by defining the JPEG-decompression for the received images
* @version 1.0
*/

#ifndef BackendServer_JPEGDecompression_h
#define BackendServer_JPEGDecompression_h
extern "C"{
#include <jpeglib.h>
}
#include "DecompressionStrategy.h"

// This is based on the FILE source manager in libjpeg

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
* @brief Class that extends the abstract class DecompressionStrategy. Offers the functionality to decompress a 12-bit JPEG-image to a depth matrix 
*/
class JPEGDecompression:public DecompressionStrategy{
public:

	/**
	*
	* @brief Public method that offers the functionality to compress a depth matrix to a 12-bit JPEG-image
	* @param[in] xResolution The horizontal resolution of the used camera
	* @param[in] yResolution The vertical resolution of the used camera
	* @param[in] depthData A pointer to a depthmatrix containing the depth data
	* @param[out] out_buffer A pointer to a databuffer containing the compressed image
	* @param[out] nBytes The size of the out_buffer in bytes     */
	void decompress_image_to_depth(const int& xResolution,const int& yResolution,short*& data,unsigned char* image){
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
