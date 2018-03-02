#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <string>
#include <jpeglib.h>
#include <jerror.h>
#include "Poco/FileStream.h"
#include "Poco/DateTime.h"
#include "Poco/DateTimeFormat.h"
#include "Poco/DateTimeFormatter.h"
#include "Poco/Timespan.h"
#include "Poco/BinaryWriter.h"


class Utilities
{
private:
	METHODDEF(void) init_source (j_decompress_ptr cinfo)
	{
	}
 
	METHODDEF(boolean) fill_input_buffer (j_decompress_ptr cinfo)
	{
	  struct jpeg_source_mgr * src = cinfo->src;
	  static JOCTET FakeEOI[] = { 0xFF, JPEG_EOI };
 
	  /* Generate warning */
	  //WARNMS(cinfo, JWRN_JPEG_EOF);
 
	  /* Insert a fake EOI marker */
	  src->next_input_byte = FakeEOI;
	  src->bytes_in_buffer = 2;
 
	  return TRUE;
	}
 
	METHODDEF(void) skip_input_data (j_decompress_ptr cinfo, long num_bytes)
	{
	  struct jpeg_source_mgr * src = cinfo->src;
 
	  if(num_bytes >= (long)src->bytes_in_buffer)
	  {
		fill_input_buffer(cinfo);
		return;
	  }
 
	  src->bytes_in_buffer -= num_bytes;
	  src->next_input_byte += num_bytes;
	}
 
	METHODDEF(void) term_source (j_decompress_ptr cinfo)
	{
	  /* no work necessary here */
	}

public:
	static std::string replaceChars(std::string s, char toReplace, char by)
	{
		char * t = new char[s.length()+1];
		strcpy(t,s.c_str());
		int i=0;
		for(i=0; i<s.length(); i++)
		{
			if(t[i]==toReplace)
				t[i]=by;
		}
		s=std::string(t);
		delete[] t;
		return s;
	}

	static void jpegToImage(char * jpeg, int size, Image** pRes)
	{
		Image * im = new Image();
		*pRes = im;

		//Declare JPEG structure
		struct jpeg_source_mgr jsrc;
		struct jpeg_decompress_struct cinfo;
		struct jpeg_error_mgr jerr;
 
		//Initialize JPEG source
		jsrc.bytes_in_buffer=size;
		jsrc.next_input_byte=(JOCTET*)jpeg;
 
		//Setup function pointer
		jsrc.init_source=init_source;
		jsrc.fill_input_buffer = fill_input_buffer;
		jsrc.skip_input_data = skip_input_data;
		jsrc.resync_to_restart = jpeg_resync_to_restart; /* use default method */
		jsrc.term_source = term_source;
 
		//Initialize JPEG structure.
		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_decompress(&cinfo);
 
		//Setup JPEG
		cinfo.src=&jsrc;
 
		//read the source datastream header markers
		jpeg_read_header(&cinfo, TRUE);
		jpeg_start_decompress(&cinfo);
		
		im->initialize(cinfo.image_width, cinfo.image_height, cinfo.num_components);
		//std::cout << im->width << ", " << im->height << " / " << im->nbChannels << std::endl;

		JSAMPROW row_pointer[1];
		row_pointer[0] = (unsigned char *)malloc( cinfo.output_width*cinfo.num_components );
		/* read one scan line at a time */
		int location=0;
		while( cinfo.output_scanline < cinfo.image_height )
		{ 
			jpeg_read_scanlines( &cinfo, row_pointer, 1 ); 
			for(int i=0; i<cinfo.image_width*cinfo.num_components;i++)   
				im->data[location++] = row_pointer[0][i];
		}
		/* wrap up decompression, destroy objects, free pointers and close open files */
		jpeg_finish_decompress( &cinfo );
		jpeg_destroy_decompress( &cinfo );
		free( row_pointer[0] );
	}

	static void RGBToPPM(std::string name, char * buffer, int size)
	{
		Poco::FileOutputStream fo(name,std::ios::out);
		Poco::BinaryWriter bw(fo);
		bw.writeRaw("P6\n640 480\n255\n",15);
		bw.writeRaw(buffer, size);
		bw.flush();
		fo.close();
	}
	
	static void deletePtr(void ** ptr) { if(*ptr!=NULL) { delete *ptr; *ptr=NULL; } }
	static void deletePtrTab(void ** ptr) { if(*ptr!=NULL) { delete [] *ptr; *ptr=NULL; } }

	static void profile(std::string name) { 
		static Poco::DateTime now(0);
		
		Poco::DateTime zero(0);
		if(now==zero) 
		{
			Poco::DateTime newnow;
			now = newnow;
		}
		else
		{
			Poco::DateTime afterNow;
			Poco::Timespan time = afterNow - now;	
			std::cout << "Profiling " << name << " : " << time.milliseconds() << " ms" << std::endl;
			now = zero;
		}
	}

	static int find(char * buffer, int size, char * string, int sizeString)
	{
		int res = -1;

		for(int i=0; i<size-sizeString; i++)
		{
			int nb=0;
			for(int j=0; j<sizeString; j++)
			{
				if(string[j]==buffer[i+nb])
				{
					nb++;
				}
				else
					break;
			}
			if(nb == sizeString)
			{
				res = i;
				break;
			}
		}

		return res;
	}
};

#endif
