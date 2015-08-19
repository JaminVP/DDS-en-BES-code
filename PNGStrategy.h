/**
* @file PNGStrategy.h
* @author Jamin Van Parys (jaminvp@gmail.com)
* @ingroup DimensionalDataSource
* @date 16/03/2015
* @brief Class of DimensionalDataSource which handles 16-bit PNG-compression of single depthframe
* @version 1.0
*/

#ifndef DimensionalDataSource_PNGStrategy_h
#define DimensionalDataSource_PNGStrategy_h

#include "CompressionStrategy.h"
#include <png.h>
#include <sstream>
#include <fstream>

using namespace std;

/* structure to store PNG image bytes */
struct mem_encode
{
	char *buffer;
	size_t size;
};

void
	my_png_write_data(png_structp png_ptr, png_bytep data, png_size_t length)
{
	/* with libpng15 next line causes pointer deference error; use libpng12 */
	struct mem_encode* p=(struct mem_encode*)png_get_io_ptr(png_ptr); /* was png_ptr->io_ptr */
	size_t nsize = p->size + length;

	/* allocate or grow buffer */
	if(p->buffer)
		p->buffer = (char*)realloc(p->buffer, nsize);
	else
		p->buffer = (char*)malloc(nsize);

	if(!p->buffer)
		png_error(png_ptr, "Write Error");

	/* copy new bytes to end of buffer */
	memcpy(p->buffer + p->size, data, length);
	p->size += length;
}




#define PNGSIGSIZE 8


bool validate(char* image,int nBytes ) {
	istringstream source(string(image,nBytes));
	//cout<<"Address of imagebuffer:"<<&image<<endl;
	//Allocate a buffer of 8 bytes, where we can put the file signature.
	png_byte pngsig[PNGSIGSIZE];
	int is_png =0;
	//Read the 8 bytes from the stream into the sig buffer.

	source.read((char*)pngsig, PNGSIGSIZE);


	//Let LibPNG check the sig. If this function returns 0, everything is OK.
	is_png = png_sig_cmp(pngsig, 0, PNGSIGSIZE);
	return (is_png == 0);
}

void readDataFromImage(png_structp pngPtr, png_bytep data, png_size_t length) {
	//cout<<"Current address of destination: "<<&data<<endl;
	//cout<<"Length of bytes to read: "<<length<<endl;

	//Here we get our IO pointer back from the read struct.
	//This is the parameter we passed to the png_set_read_fn() function.
	//Our std::istream pointer.
	istream* a =(istream*) png_get_io_ptr(pngPtr);
	//Cast the pointer to std::istream* and read 'length' bytes into 'data'
	a->read((char*)data, length);
	png_uint_32 b = png_get_uint_32(data);
	//cout<<b<<" "<<PNG_UINT_31_MAX<<endl;
}



/**
*
* @brief Class that extends the abstract class CompressionStrategy. Offers the functionality to compress a depth matrix to a 16-bit PNG-image. TODO
*
*/
class PNGStrategy:public CompressionStrategy{
public:

	/**
	*
	* @brief Public method that offers the functionality to compress a depth matrix to a 16-bit PNG-image
	* @param[in] xResolution The horizontal resolution of the used camera
	* @param[in] yResolution The vertical resolution of the used camera
	* @param[in] depthData A pointer to a depthmatrix containing the depth data
	* @param[out] out_buffer A pointer to a databuffer containing the compressed image
	* @param[out] nBytes The size of the out_buffer in bytes     */
	void compress_depth_to_image(const int& xResolution,const int& yResolution,short* depthData,unsigned char*& out_buffer_,int& nBytes){
		// get the depth data

		png_structp png_ptr;
		png_infop info_ptr;

		/* static */
		struct mem_encode state;

		/* initialise - put this before png_write_png() call */
		state.buffer = NULL;
		state.size = 0;

		// Initialize write structure
		png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
		if (png_ptr == NULL) {
			fprintf(stderr, "Could not allocate write struct\n");
			exit(1);
		}

		/* if my_png_flush() is not needed, change the arg to NULL */
		png_set_write_fn(png_ptr, &state, my_png_write_data,NULL);

		// Initialize info structure
		info_ptr = png_create_info_struct(png_ptr);
		if (info_ptr == NULL) {
			fprintf(stderr, "Could not allocate info struct\n");
			cin.get();
			exit(1);
		}
		// Write header (16bit grayscale)
		png_set_IHDR(png_ptr, info_ptr, xResolution, yResolution,
			16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);


		png_write_info(png_ptr, info_ptr);


		// Write image data
		for ( int y=0 ; y<yResolution ; y++) {
			png_bytep row_pointer =(png_bytep)depthData;
			png_write_row(png_ptr, row_pointer);
			depthData+=xResolution;
		}

		// End write
		png_write_end(png_ptr, NULL);
		//boost::shared_ptr<char>
		//fwrite(state.buffer, state.size,1 ,fp);
		out_buffer_=reinterpret_cast<unsigned char*>(state.buffer);
		nBytes=state.size;
		if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
		if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
	}


	/**
	*
	* @brief Public method that offers the functionality to decompress a 12-bit JPEG-image to a depth matrix 
	* @param[in] xResolution The horizontal resolution of the used camera
	* @param[in] yResolution The vertical resolution of the used camera
	* @param[in] depthData A pointer to a depthmatrix containing the depth data
	* @param[out] image A pointer to a databuffer containing the compressed image
	* @param[out] nBytes The size of the image in bytes     
	*/
	void decompress_image_to_depth(const int& xResolution,const int& yResolution,short*& depthData,unsigned char* image,int& nBytes){
		char* image_c=reinterpret_cast<char*>(image);

		if(validate(image_c,nBytes)){
			//cout<<"Starting decompression"<<endl;
			//cout<<"Valid PNG"<<endl;
			//cout<<"Creating PNG read struct"<<endl;
			//Here we create the png read struct. The 3 NULL's at the end can be used
			//for your own custom error handling functions, but we'll just use the default.
			//if the function fails, NULL is returned. Always check the return values!
			png_structp pngPtr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
			if (!pngPtr) {
				std::cerr << "ERROR: Couldn't initialize png read struct" << std::endl;
				return; //Do your own error recovery/handling here
			}
			//cout<<"Creating PNG info struct"<<endl;
			//Here we create the png info struct.
			//Note that this time, if this function fails, we have to clean up the read struct!
			png_infop infoPtr = png_create_info_struct(pngPtr);
			if (!infoPtr) {
				std::cerr << "ERROR: Couldn't initialize png info struct" << std::endl;
				png_destroy_read_struct(&pngPtr, (png_infopp)0, (png_infopp)0);
				return; //Do your own error recovery/handling here
			}

			//Here I've defined 2 pointers up front, so I can use them in error handling.
			//I will explain these 2 later. Just making sure these get deleted on error.
			png_bytep* rowPtrs = NULL;
			char* data = NULL; 


			//cout<<"Setting the read function"<<endl;
			istringstream source(string(image_c,nBytes));
			png_set_read_fn(pngPtr,(png_voidp)&source, readDataFromImage);

			png_set_sig_bytes(pngPtr,0);

			//cout<<"Reading PNG image info"<<endl;
			//Now call png_read_info with our pngPtr as image handle, and infoPtr to receive the file info.

			png_read_info(pngPtr, infoPtr);

			png_uint_32 imgWidth =  png_get_image_width(pngPtr, infoPtr);
			png_uint_32 imgHeight = png_get_image_height(pngPtr, infoPtr);

			//cout<<"Resolution: "<<imgWidth<<" x "<<imgHeight<<endl;
			//bits per CHANNEL! note: not per pixel!
			png_uint_32 bitdepth   = png_get_bit_depth(pngPtr, infoPtr);
			//Number of channels
			png_uint_32 channels   = png_get_channels(pngPtr, infoPtr);
			//Color type. (RGB, RGBA, Luminance, luminance alpha... palette... etc)
			png_uint_32 color_type = png_get_color_type(pngPtr, infoPtr);
			//cout<<"Bitdepth: "<<bitdepth<<endl;
			//cout<<"Channels: "<<channels<<endl;
			//cout<<"Color type: "<<color_type<<endl;

			//Here's one of the pointers we've defined in the error handler section:
			//Array of row pointers. One for every row.
			rowPtrs = new png_bytep[imgHeight];

			//Alocate a buffer with enough space.
			//(Don't use the stack, these blocks get big easilly)
			//This pointer was also defined in the error handling section, so we can clean it up on error.
			data = new char[imgWidth * imgHeight * bitdepth * channels / 8];

			depthData = new short[imgHeight * imgWidth];
			//This is the length in bytes, of one row.
			const unsigned int stride = imgWidth * bitdepth * channels / 8;
			//cout<<"Length of one row: "<<stride<<endl;
			//A little for-loop here to set all the row pointers to the starting
			//Adresses for every row in the buffer

			for (size_t i = 0; i < imgHeight; i++) {
				//Set the pointer to the data pointer + i times the row stride.
				//Notice that the row order is reversed with q.
				//This is how at least OpenGL expects it,
				//and how many other image loaders present the data.
				png_uint_32 q = (imgHeight- i - 1) * stride;
				rowPtrs[i] = (png_bytep)data + q;
			}
			//cout<<"Row pointers set"<<endl;

			//And here it is! The actual reading of the image!
			//Read the imagedata and write it to the adresses pointed to
			//by rowptrs (in other words: our image databuffer)
			//cout<<"Reading image"<<endl;
           	png_read_image(pngPtr, rowPtrs);

			//cout<<"Reading image done!"<<endl;
			for(int i=0;i<imgHeight;i++){
				short* ptrA = reinterpret_cast<short*>(rowPtrs[i]);
				for(int j=0;j<imgWidth;j++){
					depthData[i*imgWidth+j]=ptrA[j];
				}
			}
			//Delete the row pointers array....
			delete[] (png_bytep)rowPtrs;
			//And don't forget to clean up the read and info structs !
			png_destroy_read_struct(&pngPtr, &infoPtr,(png_infopp)0);
			//cout<<"Decompression done"<<endl;
		}
		else{
			cout<<"Unvalid PNG"<<endl;
		}
	}

};
#endif
