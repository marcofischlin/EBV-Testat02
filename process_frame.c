/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>


void ProcessFrame(uint8 *pInputImg)
{
	int c, r;
	int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);

	int Shift = 7;
	short Beta = 2;//the meaning is that in floating point the value of Beta is = 6/(1 << Shift) = 6/128 = 0.0469

	if(data.ipc.state.nStepCounter == 1)
	{
		/* this is the first time we call this function */
		/* first time we call this; index 1 always has the background image */
		memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[GRAYSCALE], sizeof(data.u8TempImage[GRAYSCALE]));
	}
	else
	{
		/* this is the default case */
		for(r = 0; r < siz; r+= nc)/* we strongly rely on the fact that them images have the same size */
		{
			for(c = 0; c < nc; c++)
			{
				/* first determine the forground estimate */
				data.u8TempImage[THRESHOLD][r+c] = abs(data.u8TempImage[GRAYSCALE][r+c]-data.u8TempImage[BACKGROUND][r+c]) < data.ipc.state.nThreshold ? 0 : 255;

				/* now update the background image; the value of background should be corrected by the following difference (* 1/128) */
				short Diff = Beta*((short) data.u8TempImage[GRAYSCALE][r+c] - (short) data.u8TempImage[BACKGROUND][r+c]);

				if(abs(Diff) >= 128) //we will have a correction - apply it (this also avoids the "bug" that -1 >> 1 = -1)
					data.u8TempImage[BACKGROUND][r+c] += (uint8) (Diff >> Shift);
				else //due to the division by 128 the correction would be zero -> thus add/subtract at least unity
				{
					if(Diff > 0 && data.u8TempImage[BACKGROUND][r+c] < 255)
							data.u8TempImage[BACKGROUND][r+c] += 1;
					else if(Diff < 0 && data.u8TempImage[BACKGROUND][r+c] > 1)
							data.u8TempImage[BACKGROUND][r+c] -= 1;
				}
			}
		}

		for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
		{
			for(c = 1; c < nc-1; c++)
			{
				unsigned char* p = &data.u8TempImage[THRESHOLD][r+c];
				data.u8TempImage[EROSION][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
												 *(p-1)    & *p      & *(p+1) &
												 *(p+nc-1) & *(p+nc) & *(p+nc+1);
			}
		}

		for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
		{
			for(c = 1; c < nc-1; c++)
			{
				unsigned char* p = &data.u8TempImage[EROSION][r+c];
				data.u8TempImage[DILATION][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
												  *(p-1)    | *p      | *(p+1) |
												  *(p+nc-1) | *(p+nc) | *(p+nc+1);
			}
		}

		//wrap image DILATION in picture struct
		Pic1.data = data.u8TempImage[DILATION];
		Pic1.width = nc;
		Pic1.height = OSC_CAM_MAX_IMAGE_HEIGHT/2;
		Pic1.type = OSC_PICTURE_GREYSCALE;
		//as well as EROSION (will be used as output)
		Pic2.data = data.u8TempImage[EROSION];
		Pic2.width = nc;
		Pic2.height = OSC_cam_max_image_height/2;
		Pic2.type = OSC_PICTURE_BINARY; //probably has no consequences
		//have to convert to OSC_PICTURE_BINARY which has values 0x01 (and not 0xff)
		OscVisGrey2BW(&Pic1, &Pic2, 0x80, false);

		//now do region labeling and feature extraction
		OscSixLabelbinary(&Pic2, &ImgRegions);
		OscVisGetRegionProperties(&ImgRegions);

		//OscLog(INFO, "number of objects %d\n", ImgRegions.noOfObjects);
		//ploit bounding boxes both in grey and dilation image
		Pic2.data = data.u8TempImage[GRAYSCALE];
		OscVisDrawBoundingBoxBW( &Pic2, &ImgRegions, 255);
		OscVisDrawBoundingBoxBW( &Pic1, &ImgRegions, 128);

	}
}


/* Drawing Function for Bounding Boxes. in Gray value Color */
OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions)
{
        uint16 i, o;
        uint8 *pImg = (uint8*)picIn->data;
        const uint16 width = picIn->width;
        for(o = 0; o < regions->noOfObjects; o++)
        {

                /* Draw the horizontal lines. */
                for (i = regions->objects[o].bboxLeft; i < regions->objects[o].bboxRight; i += 1)
                {
                        pImg[(width * regions->objects[o].bboxTop + i)]= Color;

                        pImg[(width * (regions->objects[o].bboxBottom - 1) + i)] = Color;

                }

                /* Draw the vertical lines. */

                for (i = regions->objects[o].bboxTop; i < regions->objects[o].bboxBottom-1; i += 1)
               {
                        pImg[(width * i + regions->objects[o].bboxLeft)] = Color;

                        pImg[(width * i + regions->objects[o].bboxRight)] = Color;

                }
       }
       return SUCCESS;
}
