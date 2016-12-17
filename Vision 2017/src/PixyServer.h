/*
 * PixyServer.h
 *
 *  Created on: Dec 16, 2016
 *      Author: darzins
 */

#ifndef SRC_PIXYSERVER_H_
#define SRC_PIXYSERVER_H_

#include "WPILib.h"

class PixyServer : public CameraServer {
public:

	static PixyServer* pixyInstance() {
		if (!PixyServer::m_server) {
			PixyServer::m_server = new PixyServer();
		}
		return PixyServer::m_server;
	}

	void pixy_put_frame() {

		uint8_t* videodata;
		int32_t response;
		int32_t fourccc;
		int8_t renderflags;
		uint16_t xwidth;
		uint16_t ywidth;
		uint32_t size;


		int return_value = pixy_command("cam_getFrame",  // String id for remote procedure
				INT8(0x21),     // mode
				INT16(0),        // xoffset
				INT16(0),         // yoffset
				INT16(320),       // width
				INT16(200),       // height
				END_OUT_ARGS,              // separator
				&response,      // pointer to mem address for return value
				&fourccc,
				&renderflags,
				&xwidth,
				&ywidth,
				&size,
				&videodata,        // pointer to mem address for returned frame
				END_IN_ARGS);


		if(return_value==0) {
			return_value = renderBA81(renderflags,xwidth,ywidth,size,videodata);
		}
	}

private:
	static PixyServer *m_server;

	inline void interpolateBayer(uint16_t width, uint16_t x, uint16_t y, uint8_t *pixel, uint8_t* r, uint8_t* g, uint8_t* b)
	{
		if (y&1)
		{
			if (x&1)
			{
				*r = *pixel;
				*g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
				*b = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
			}
			else
			{
				*r = (*(pixel-1)+*(pixel+1))>>1;
				*g = *pixel;
				*b = (*(pixel-width)+*(pixel+width))>>1;
			}
		}
		else
		{
			if (x&1)
			{
				*r = (*(pixel-width)+*(pixel+width))>>1;
				*g = *pixel;
				*b = (*(pixel-1)+*(pixel+1))>>1;
			}
			else
			{
				*r = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
				*g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
				*b = *pixel;
			}
		}

	}

	int renderBA81(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame)
	{
		uint16_t x, y;
		uint8_t r, g, b;

		// skip first line
		frame += width;
		uint8_t data[3*((height-2)*(width-2))];

		uint m = 0;
		    for (y=1; y<height-1; y++)
		    {
		        frame++;
		        for (x=1; x<width-1; x++, frame++)
		        {
		            interpolateBayer(width, x, y, frame, &r, &g, &b);
		            data[m++] = b;
		            data[m++] = g;
		            data[m++] = r;
		        }
		        frame++;
		    }


		SetImageData(data, (width-2)*(height-2));

		return 0;
	}

};

PixyServer* PixyServer:: m_server = NULL;


#endif /* SRC_PIXYSERVER_H_ */
