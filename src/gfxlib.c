/**
* ****************************************************************************************************************
*  @file    gfxlib.c
*  @author  Markus Frejek; somewhat based on Uwe Becker's ST7783 library http://mikrocontroller.bplaced.net/wordpress/?page_id=211; and Andy Brown's STM32Plus libraries
*  @date    19.01.2018
*  @brief   ILI9481 display driver + graphics library main file
*/


#include "gfxlib.h"
#include "ili9481_cmd.h"
#include "font1632.h"
#include "font1224.h"
#include "font1016.h"
#include "font0507.h"

/* Frame duration:
Without DMA: 33.6 ms
With DMA + 2-dim array: 23.7 ms
With DMA + 1 dim array: 22.6 ms
DMA + 32bit erase: 21.4 ms
Frame with 100 polygons: 34.6 ms
*/

#ifdef DMA_ENABLE
  volatile uint16_t bufindex = 0;
#endif
uint16_t ypos;


uint16_t currline[CURRLINE_LEN];
//--------------------------------------------------------------

struct gfxItem graphItems[NUM_GFX_ELEMENTS];	//Graphics buffer with n elements
uint16_t uGfxUsed;	//Num of used GFX structs
uint32_t bgColor32;	//Background color

volatile uint32_t benchtime;

/*********************   LOCAL FUNCTIONS  ********************************/

/**
 * "5 bit" lookup table for the offsets. These are the sines for exactly
 * at 0deg, 11.25deg, 22.5deg etc. The values are from -1 to 1 in Q15.
 */
static int16_t sin90[TABLE_SIZE+1] = {
  0x0000,0x0647,0x0c8b,0x12c7,0x18f8,0x1f19,0x2527,0x2b1e,
  0x30fb,0x36b9,0x3c56,0x41cd,0x471c,0x4c3f,0x5133,0x55f4,
  0x5a81,0x5ed6,0x62f1,0x66ce,0x6a6c,0x6dc9,0x70e1,0x73b5,
  0x7640,0x7883,0x7a7c,0x7c29,0x7d89,0x7e9c,0x7f61,0x7fd7,
  0x7fff
};

//Check if the gradient of a line is positive or negative
static inline uint8_t checkGradient(int16_t py, int16_t delta)
{
	if((py > 0) && (delta > 0)) return 1;
	if((py < 0) && (delta < 0)) return 1;
	return 0;
}

// Sorting function; needed for drawing polygons
void GnomeSort2(struct sortItem list[], uint16_t length){
	uint16_t pos = length;
	while((pos > 0) && (list[pos-1].xpos > list[pos].xpos)) {
		struct sortItem tmp = list[pos];
		list[pos] = list[pos-1];
		list[pos-1] = tmp;
		pos--;
	}
}
// Sorting function; needed for drawing polygons
void GnomeSort(struct sortItem list[], uint16_t length){
	for(uint16_t i=0;i<length;i++) GnomeSort2(list,i);
}

/** Sine/cosine calculation functions
 * Source: https://www.atwillys.de/content/cc/sine-lookup-for-embedded-in-c/?lang=en
 * Sine calculation using interpolated table lookup.
 * Instead of radiants or degrees we use "turns" here. Means this
 * sine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * See the full description at www.AtWillys.de for the detailed
 * explanation.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
static int16_t sin1(int16_t angle)
{
  int16_t v0, v1;
  if(angle < 0) { angle += INT16_MAX; angle += 1; }
  v0 = (angle >> INTERP_BITS);
  if(v0 & FLIP_BIT) { v0 = ~v0; v1 = ~angle; } else { v1 = angle; }
  v0 &= TABLE_MASK;
  v1 = sin90[v0] + (int16_t) (((int32_t) (sin90[v0+1]-sin90[v0]) * (v1 & INTERP_MASK)) >> INTERP_BITS);
  if((angle >> INTERP_BITS) & NEGATE_BIT) v1 = -v1;
  return v1;
}

/** Source: https://www.atwillys.de/content/cc/sine-lookup-for-embedded-in-c/?lang=en
 * Cosine calculation using interpolated table lookup.
 * Instead of radiants or degrees we use "turns" here. Means this
 * cosine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
static int16_t cos1(int16_t angle)
{
  if(angle < 0) { angle += INT16_MAX; angle += 1; }
  return sin1(angle - (270L * INT16_MAX) / 360);
}

/**
* @brief Calculate an angle from X and Y difference to center and radius. Works in entire 360 deg range, like the standard atan2 function
* @param deltaY: Y distance from center point
* @param deltaX: X distance from center point
* @param radius: Distance of point given by deltaY and deltaX to center point
* @return angle in Q15 format
*/
static int16_t asin21(int16_t deltaY, int16_t deltaX, int16_t radius)
{
	//Find nearest sine from table
	int8_t indexBelow, invert = 0;
	int16_t tmpangle = 0;
	int16_t sinval = (32767L * deltaY) / radius;
	if(sinval < 0) 
	{
		sinval = -sinval;
		invert = 1;
	}
  int8_t indexAbove = BINARY_MID;
  int8_t testinc = BINARY_INC;
  
	while(testinc > 0) {
		if(sin90[indexAbove] > sinval) indexAbove -= testinc;
		else if(sin90[indexAbove] < sinval) indexAbove += testinc;
    testinc >>= 1;
	}
  indexBelow = indexAbove-1;
	if(indexBelow < 0) indexBelow = 0;
	//tmpangle += sin90[indexBelow] + ((int32_t)(sinval - sin90[indexBelow]) * (16383 / TABLE_SIZE)) / (sin90[indexAbove] - sin90[indexBelow]);
	tmpangle = (8191L / TABLE_SIZE) * indexBelow;
	tmpangle += (((8191L / TABLE_SIZE) * ((int32_t)(sinval - sin90[indexBelow]))) / (sin90[indexAbove] - sin90[indexBelow]));
	if(deltaX < 0) tmpangle = 16383 - tmpangle;
	if(invert == 1) tmpangle += 16384;
	return tmpangle;
}

/**
* @brief Set a pixel in the currently drawn line. Don't call this function externally
* @param pos: Pixel to draw in buffer (X coordinate + buffer index)
* @param color: RGB 565 color for the pixel
*/
static inline void GFXDrawLinePixel(uint16_t pos, uint16_t color) {
  currline[pos] = color;
}


/**** INTERNAL LINE DRAWING FUNCTIONS *****/

/**
* @brief Internal function for drawing a line of a text element. Called from GFXUpdateDisplay(); don't call this from anywhere else.
* @param rel_y: Y position of line to draw relative to item top
* @param itemnum: Index of graphics item to draw
* @retval none
*/
static inline void GFXDrawTextLine(uint16_t rel_y, uint16_t itemnum) {
	
	uint16_t textWidth = 0;//graphItems[itemnum].graphData.dispText.length;
	
	for(uint16_t i=0;i<graphItems[itemnum].graphData.dispText.length;i++) {
		if(graphItems[itemnum].graphData.dispText.textHeight == 16) {
			//10x16 font engine
			for(uint16_t j=512;j>0;j/=2) {
				if(font1016[graphItems[itemnum].graphData.dispText.drawText[i]][rel_y] & j) {
					GFXDrawLinePixel(textWidth + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].fgColor);
				} else {
					if(!(graphItems[itemnum].graphData.dispText.drawMode & 1)) GFXDrawLinePixel(textWidth + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].bgColor);
				}
				textWidth++;
			}
			/*
			// Alternative: Use every second pixel of 16x32 for a 8x16 font
			for(uint8_t j=16;j>0;j-=2) {
				if(font1632[((graphItems[itemnum].graphData.dispText.drawText[i] - 32) * 32) + (rel_y * 2)] & (1<<(j-2))) {
					currline[textWidth + graphItems[itemnum].xpos] = color;
					//currline[xindex - 1] = 0x1F;
				} else {
					if(graphItems[itemnum].graphData.dispText.drawMode & 1) {
						currline[textWidth + graphItems[itemnum].xpos] = graphItems[itemnum].bgColor;
					}
				}
				textWidth++;
			}
			*/
		} else if(graphItems[itemnum].graphData.dispText.textHeight == 24) {
			//12x24 font engine
			for(uint16_t j=32768;j>4;j/=2) {
				if(font1224[((graphItems[itemnum].graphData.dispText.drawText[i]) * 24) + rel_y] & j) {
					GFXDrawLinePixel(textWidth + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].fgColor);
					//currline[xindex - 1] = 0x1F;
				} else {
					if(!(graphItems[itemnum].graphData.dispText.drawMode & 1)) {
						GFXDrawLinePixel(textWidth + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].bgColor);
					}
				}
				textWidth++;
			}
		} else if(graphItems[itemnum].graphData.dispText.textHeight == 32) {
			//16x32 font engine
			for(uint16_t j=32768;j>0;j/=2) {
				if(font1632[((graphItems[itemnum].graphData.dispText.drawText[i] - 32) * 32) + rel_y] & j) {
					GFXDrawLinePixel(textWidth + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].fgColor);
					//currline[xindex - 1] = 0x1F;
				} else {
					if(!(graphItems[itemnum].graphData.dispText.drawMode & 1)) {
						GFXDrawLinePixel(textWidth + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].bgColor);
					}
				}
				textWidth++;
			}
		} else if(graphItems[itemnum].graphData.dispText.textHeight == 8) {
			//5*8 font engine
			for(uint8_t j=0;j<5;j++) {
				if(font57[graphItems[itemnum].graphData.dispText.drawText[i]][j] & (1<<rel_y)) {
					GFXDrawLinePixel(textWidth + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].fgColor);
					//currline[xindex - 1] = 0x1F;
					} else {
					if(!(graphItems[itemnum].graphData.dispText.drawMode & 1)) GFXDrawLinePixel(textWidth + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].bgColor);
				}
				textWidth++;
			}
			if(!(graphItems[itemnum].graphData.dispText.drawMode & 1)) GFXDrawLinePixel(textWidth + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].bgColor);
			textWidth++;
		}
	}
}

/**
* @brief Internal function for drawing a line of a rectangle/bitmap element. Called from GFXUpdateDisplay(); don't call this from anywhere else.
* @param rel_y: Y position of line to draw relative to item top
* @param itemnum: Index of graphics item to draw
* @retval none
*/
static inline void GFXDrawRectLine(uint16_t rel_y, uint16_t itemnum) {
	// Add all pixels of rect to output line
  if((graphItems[itemnum].graphData.dispRect.fillStyle & 2) == 0) { // Check if element is rectangle or bitmap
    //Graphics element is rectangle
    if(graphItems[itemnum].graphData.dispRect.fillStyle & 8) //Check if 32bit access possible
    {
      // 32 bit access
      uint32_t bgColor32b = ((uint32_t)graphItems[itemnum].bgColor << 16) + graphItems[itemnum].bgColor;
      uint32_t fgColor32b = ((uint32_t)graphItems[itemnum].fgColor << 16) + graphItems[itemnum].fgColor;
      uint32_t* px = (uint32_t*)&currline[graphItems[itemnum].xpos + bufindex];
      for(uint16_t i=0;i<graphItems[itemnum].graphData.dispRect.width;i+=2) {
        // Check if pixel is in border
        if((i >= graphItems[itemnum].graphData.dispRect.borderWidth) && (i<= graphItems[itemnum].graphData.dispRect.width - graphItems[itemnum].graphData.dispRect.borderWidth - 1) && (rel_y >= graphItems[itemnum].graphData.dispRect.borderWidth) && (rel_y < graphItems[itemnum].graphData.dispRect.height - graphItems[itemnum].graphData.dispRect.borderWidth)) {
          //Inside rectangle
          if(graphItems[itemnum].graphData.dispRect.fillStyle & 1) {
            // Skip to end of inside if transparency is enabled
            i = graphItems[itemnum].graphData.dispRect.width - graphItems[itemnum].graphData.dispRect.borderWidth;
            px = (uint32_t*)&currline[graphItems[itemnum].xpos + bufindex + i];
          } else {
           // Fill rect pixel if it should be filled, defined by fill style
            *px++ = fgColor32b;
          }
        } else {
          //Inside border
          *px++ = bgColor32b;
        }
      }
    } else {
      //16 bit access needed
      for(uint16_t i=0;i<graphItems[itemnum].graphData.dispRect.width;i++) {
        // Check if pixel is in border
        if((i >= graphItems[itemnum].graphData.dispRect.borderWidth) && (i<= graphItems[itemnum].graphData.dispRect.width - graphItems[itemnum].graphData.dispRect.borderWidth - 1) && (rel_y >= graphItems[itemnum].graphData.dispRect.borderWidth) && (rel_y < graphItems[itemnum].graphData.dispRect.height - graphItems[itemnum].graphData.dispRect.borderWidth)) {
          //Inside rectangle
          if(graphItems[itemnum].graphData.dispRect.fillStyle & 1) {
            // Skip to end of inside if transparency is enabled
            i = graphItems[itemnum].graphData.dispRect.width - graphItems[itemnum].graphData.dispRect.borderWidth;
          } else {
            // Fill rect pixel if it should be filled, defined by fill style
            GFXDrawLinePixel(i + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].fgColor);
          }
        } else {
          //Inside border
          GFXDrawLinePixel(i + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].bgColor);
        }
      }
    }
  } else {
    //Graphics element is bitmap
    if((graphItems[itemnum].graphData.dispRect.fillStyle & 4) == 0) { // Check if 1 bpp or 16 bpp
      //16 bpp
      if(graphItems[itemnum].graphData.dispRect.fillStyle & 1) //Check if transparency enabled
      { //Transparency enabled => Pixels must be checked + set one by one
        uint32_t bmppixel = (uint32_t)graphItems[itemnum].graphData.dispRect.width * rel_y; //Pixel number in bitmap
        for(uint16_t i=0;i<graphItems[itemnum].graphData.dispRect.width;i++) 
        {
          if(graphItems[itemnum].bgColor != graphItems[itemnum].graphData.dispRect.bitmap_data[bmppixel]) // If transparency: Draw pixel only if its color isn't the bitmap's transparent color
          {
            GFXDrawLinePixel(i + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].graphData.dispRect.bitmap_data[bmppixel]);
          }
          bmppixel++;
        }
      } else {  //No transparency => 32 bit access possible for speed
        if(graphItems[itemnum].graphData.dispRect.fillStyle & 8) //Check if 32bit access possible
        {
          uint32_t* px = (uint32_t*)&currline[graphItems[itemnum].xpos + bufindex];
          uint32_t* bmppixel = (uint32_t*)(&graphItems[itemnum].graphData.dispRect.bitmap_data[(graphItems[itemnum].graphData.dispRect.width * rel_y)]); //Pixel number in bitmap
          for(uint16_t i=0;i<graphItems[itemnum].graphData.dispRect.width;i+=2) 
          {
            *px++ = *bmppixel++;
          }
        } else {
          uint32_t bmppixel = (uint32_t)graphItems[itemnum].graphData.dispRect.width * rel_y; //Pixel number in bitmap
          for(uint16_t i=0;i<graphItems[itemnum].graphData.dispRect.width;i++) 
          {
            GFXDrawLinePixel(i + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].graphData.dispRect.bitmap_data[bmppixel++]);
          }
        }
      }
    } else {
      //1 bpp
      uint32_t startpixel = (uint32_t)graphItems[itemnum].graphData.dispRect.width * rel_y; //Pixel number in bitmap
      uint32_t bmppixel = startpixel; //Pixel number in bitmap
      uint32_t endpixel = (uint32_t)graphItems[itemnum].graphData.dispRect.width * (rel_y + 1); //End Pixel number in bitmap
      uint32_t testpx = graphItems[itemnum].graphData.dispRect.bitmap_data[bmppixel >> 4];  //Single bitmap pixel
      uint32_t pxoffset = graphItems[itemnum].xpos + bufindex;
      if(graphItems[itemnum].graphData.dispRect.fillStyle & 1)  //Transparency enabled => Draw only '1' pixels
      {
        while(bmppixel < endpixel) 
        {
          //Transparency enabled => Don't draw '0' pixels
          uint32_t pxbit = (bmppixel & 0x0F);
          if(testpx & (1 << pxbit)) //Check if pixel is 1 or 0
          {
            // Pixel is 1 => Draw in foreground color
            GFXDrawLinePixel(pxoffset + (bmppixel - startpixel), graphItems[itemnum].fgColor);
          }
          bmppixel++;
          if(pxbit == 15)  //Read next pixel
          {
            do {
              testpx = graphItems[itemnum].graphData.dispRect.bitmap_data[bmppixel >> 4];  //Single bitmap pixel
              if(testpx == 0) {
                bmppixel += 16;
                if(bmppixel > endpixel) return;
              }
            } while(testpx == 0);
          }
        }
      } else {
        for(uint16_t i=0;i<graphItems[itemnum].graphData.dispRect.width;i++) 
        {
          //Transparency not enabled => Draw '0' pixels in background color
          if(graphItems[itemnum].graphData.dispRect.bitmap_data[bmppixel >> 4] & (1 << (bmppixel & 0x0F))) //Check if pixel is 1 or 0
          {
            // Pixel is 1 => Draw in foreground color
            GFXDrawLinePixel(i + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].fgColor);
          } else {
            // Pixel is 0 => Draw in background color
             GFXDrawLinePixel(i + graphItems[itemnum].xpos + bufindex, graphItems[itemnum].bgColor);
          }
          bmppixel++;
        }
      }
    }
  }
}

/**
* @brief Internal function for drawing a line of a circle element. Called from GFXUpdateDisplay(); don't call this from anywhere else.
* @param rel_y: Y position of line to draw relative to item top
* @param itemnum: Index of graphics item to draw
* @retval none
*/
static inline void GFXDrawCircleLine(uint16_t rel_y, uint16_t itemnum) {
	// Add all pixels of circle to output line
  uint16_t inend = 0;     //Last line pixel of circle inside
  uint16_t borderend = 0; //Last line pixel of circle border
  //Calculate dy� for radius 
  int32_t ydiv2 = ((int16_t)graphItems[itemnum].graphData.dispCircle.radius - (int16_t)rel_y) * ((int16_t)graphItems[itemnum].graphData.dispCircle.radius - (int16_t)rel_y);
	//Draw circle until inside of border is reached
  uint16_t px = (int16_t)graphItems[itemnum].xpos + bufindex;
  //uint16_t endx = (int16_t)graphItems[itemnum].xpos + 2 * graphItems[itemnum].graphData.dispCircle.radius + bufindex;
  //Find first intersection of line with border and circle inside
  for(int16_t i=0;i<graphItems[itemnum].graphData.dispCircle.radius;i++) {
		// Calculate square of distance between pixel and center: dx� + dy�
		int32_t pointrad2 = (i - (int16_t)graphItems[itemnum].graphData.dispCircle.radius) * (i - (int16_t)graphItems[itemnum].graphData.dispCircle.radius) + ydiv2;
		// Check if pixel is in circle: dx� + dy� <= r�
		if(pointrad2 <= graphItems[itemnum].graphData.dispCircle.rad2) {
      if(borderend == 0) borderend = bufindex + (int16_t)graphItems[itemnum].xpos + (2 * graphItems[itemnum].graphData.dispCircle.radius - i);
      if(ydiv2 > graphItems[itemnum].graphData.dispCircle.inrad2) {
        GFXDrawLinePixel(px, graphItems[itemnum].bgColor);
        continue;
      }
			//Check if also inside of border
			if(pointrad2 <= graphItems[itemnum].graphData.dispCircle.inrad2) {
				//Inside border
        if(inend == 0) inend = bufindex + (int16_t)graphItems[itemnum].xpos + (2 * graphItems[itemnum].graphData.dispCircle.radius - i);
        break;
			} else {
				//Pixel is part of border
				GFXDrawLinePixel(px, graphItems[itemnum].bgColor);
			}
		}
    px++;
	}
  if(graphItems[itemnum].graphData.dispCircle.fillStyle & 1) {
    while(px < inend) {
      GFXDrawLinePixel(px, graphItems[itemnum].fgColor);
      px++;
    }
  }
  if(inend != 0) px = inend;
  while(px < borderend) {
    GFXDrawLinePixel(px, graphItems[itemnum].bgColor);
    px++;
  }
}

/**
* @brief Internal function for drawing a line of a circle element. Called from GFXUpdateDisplay(); don't call this from anywhere else.
* @param rel_y: Y position of line to draw relative to item top
* @param itemnum: Index of graphics item to draw
* @retval none
*/
static inline void GFXDrawArcLine(uint16_t rel_y, uint16_t itemnum) {
	// Add all pixels of arc to output line
	//TEST
	int16_t testangle;
	uint8_t angleCalcd = 0;
	uint8_t drawActive = 0;
	int32_t px1, px2;
	int32_t pointr;
	int16_t ydiv = (int16_t)graphItems[itemnum].graphData.dispArc.radius - (int16_t)rel_y;
  int32_t ydiv2 = ydiv * ydiv;
	//Calculate intersection points for start and end
	px1 = ((int32_t)ydiv * (int32_t)graphItems[itemnum].graphData.dispArc.px[0]) / graphItems[itemnum].graphData.dispArc.py[0];
	px2 = ((int32_t)ydiv * (int32_t)graphItems[itemnum].graphData.dispArc.px[1]) / graphItems[itemnum].graphData.dispArc.py[1];
	/* Check if there is something to draw at all */
	for(int16_t xdiv=-graphItems[itemnum].graphData.dispArc.radius; xdiv<graphItems[itemnum].graphData.dispArc.radius; xdiv++) {
		// Calculate X and Y relative to center
		// Check if pixel is in circle (arc thought to be 360�): dx� + dy� <= r�
		pointr = ((int32_t)xdiv * (int32_t)xdiv) + ydiv2;
		if((pointr <= graphItems[itemnum].graphData.dispArc.rad2) &&
		(pointr >= graphItems[itemnum].graphData.dispArc.inrad2)) {
			if(angleCalcd == 0)
			{
				angleCalcd = 1;
				testangle = asin21(ydiv, xdiv, graphItems[itemnum].graphData.dispArc.radius);
				if((testangle >= graphItems[itemnum].graphData.dispArc.startAngle) && (testangle < graphItems[itemnum].graphData.dispArc.endAngle)) {
					drawActive = 1;
				}
			}
			if((xdiv > px2) && checkGradient(graphItems[itemnum].graphData.dispArc.py[1], ydiv))
			{
				drawActive = 1;
				px2 = 1000000;
			}
			if((xdiv > px1) && checkGradient(graphItems[itemnum].graphData.dispArc.py[0], ydiv))
			{
				drawActive = 0;
				px1 = 1000000;
			}
			//if(xdiv >= px1) drawActive = 1;
			//if(xdiv >= px2) drawActive = 0;
			if(drawActive == 1) {
        int16_t tmpx = xdiv + graphItems[itemnum].graphData.dispArc.radius + graphItems[itemnum].xpos;
				if((tmpx >= 0) && (tmpx <= SCREEN_WIDTH)) GFXDrawLinePixel(tmpx + bufindex, graphItems[itemnum].fgColor);
			}
		}
	}
}

/**
* @brief Internal function for drawing a line of a rectangle/bitmap element. Called from GFXUpdateDisplay(); don't call this from anywhere else.
* @param rel_y: Y position of line to draw relative to item top
* @param itemnum: Index of graphics item to draw
* @retval none
*/
static inline void GFXDrawPolyLine(uint16_t rel_y, uint16_t itemnum) {
	/* Variables for the crossing points */
	struct sortItem crossings[MAX_POLY_POINTS];
	int16_t x0, x1, y1, x2, y2;
  uint16_t x;
	x0 = graphItems[itemnum].xpos;
	uint8_t crossingnum = 0;
	/* Iterate over all polygon points */
	for(uint8_t i=0;i<graphItems[itemnum].bgColor;i++) {
		// Check if last element. If yes, draw line between first and last element to close the polygon
		if(i > 0) {
			x1 = graphItems[itemnum].graphData.dispPoly.px[i-1];
			y1 = graphItems[itemnum].graphData.dispPoly.py[i-1];
			x2 = graphItems[itemnum].graphData.dispPoly.px[i];
			y2 = graphItems[itemnum].graphData.dispPoly.py[i];
		} else {
			x1 = graphItems[itemnum].graphData.dispPoly.px[graphItems[itemnum].bgColor-1];
			y1 = graphItems[itemnum].graphData.dispPoly.py[graphItems[itemnum].bgColor-1];
			x2 = graphItems[itemnum].graphData.dispPoly.px[0];
			y2 = graphItems[itemnum].graphData.dispPoly.py[0];
		}
		/* Check if the line between both points even crosses the currently drawn line */
		if(((y1 <= rel_y) && (y2 >= rel_y)) || ((y1 >= rel_y) && (y2 <= rel_y))) {
			/* Line between points crosses draw line */
			if(y1 == y2) {
				/* Special case: Line between points is horizontal and exactly on draw line => Draw entire line between points*/
				x1 += x0;
				x2 += x0;
				for(x=x1; x<=x2; x++) GFXDrawLinePixel(x + bufindex, graphItems[itemnum].fgColor);
			} else {
				// Determine + store grade of line (upwards/downwards) 
				if(((int16_t)y2 - y1) > 0) { 
          crossings[crossingnum].tangent = 1;
        } else crossings[crossingnum].tangent = 0;
				crossings[crossingnum].xpos = (uint32_t)x0 + x1 + (int32_t)((int32_t)(x2 - x1) * ((int32_t)rel_y - y1)) / ((int32_t)y2 - y1);
				crossingnum++;
			}
		}
	}
	if(crossingnum > 0) {
		//Handle the crossings
		//Sort them with ascending X ccords
		GnomeSort(crossings, crossingnum);
		uint8_t i = 0;
		int16_t crossing_cmp = crossingnum;
		//if(crossing_cmp & 1) crossing_cmp--;
		while(i<crossing_cmp-1) {
			if(i >= crossing_cmp) break;
			x1 = crossings[i].xpos;
			x2 = crossings[i+1].xpos;
			y1 = crossings[i].tangent;
			y2 = crossings[i+1].tangent;
			// If two crossings have the same X pos and gradient, they are just a bend in the line. In this case, handle them as only one crossing
			if((x1 == x2) && (y1 == y2)) {
				i++;
				continue;
			}
			for(x=x1; x<=x2; x++) GFXDrawLinePixel(x + bufindex, graphItems[itemnum].fgColor);
			i += 2;
		}
	}
}

/**
* @brief Check if a line crosses a graphics element
* @param y: Screen line number
* @param itemnum: Number of item to check
* @retval: none
*/
static inline uint8_t GFXLineIsInItem(uint16_t y, uint16_t itemnum) {
	if((y >= graphItems[itemnum].ypos) && (y < graphItems[itemnum].yend)) return 1;
	return 0;
}

/**** PUBLIC DRAWING FUNCTIONS ****

* This are the functions you want to call from your application

*/

/**
* @brief Add a text element on LCD
* @param itemnum: # of element to edit, -1 to add a new one
* @param text: max. 33 chars long, + terminating null
* @param x: X pos (upper left corner) of text on screen
* @param Y: Y pos (upper left corner) of text on screen
* @param color: RGB 565 color
* @param size: Font size; XSMALL(8px), SMALL(16px), MEDIUM(24px), or LARGE(32px)
* @param bgTransparent: 1 = background transparent; 0 = background in bgColor
* @param bgColor: Bachground color; irgnored if bgTransparent=1
* @retval If function fails: -1 (all available graphics elemts occupied or text too long), on success: # of graphics element
*/

int16_t GFXAddText(int16_t itemnum, char* text, uint16_t x, uint16_t y, uint16_t color, enum FontSize size, uint8_t bgTransparent, uint16_t bgColor) {
  uint8_t fontWidth[4] = {5, 10, 12, 16}; //Width in px of the 4 fonts
  if(itemnum == -1)
  {
    // Create new element
    if(uGfxUsed >= NUM_GFX_ELEMENTS) return -1;	//Abort if no further GFX elements are available
    itemnum = uGfxUsed;
    uGfxUsed++;
  }
	uint8_t len;
  for(len=0;len<33;len++) {
    if(text[len] < ' ') {
      graphItems[itemnum].graphData.dispText.drawText[len] = ' ';
    } else {
      graphItems[itemnum].graphData.dispText.drawText[len] = text[len];
    }
    if(text[len] == 0) break;
  }
  if((x + (fontWidth[size] * len)) > SCREEN_WIDTH) return -1; //Prevent exceeding screen width
	// Set parameters
	graphItems[itemnum].graphData.dispText.drawMode = bgTransparent;
	graphItems[itemnum].graphData.dispText.length = len;
	if(size == XSMALL) graphItems[itemnum].graphData.dispText.textHeight = 8;
	else if(size == SMALL) graphItems[itemnum].graphData.dispText.textHeight = 16;
	else if(size == MEDIUM) graphItems[itemnum].graphData.dispText.textHeight = 24;
	else if(size == LARGE) graphItems[itemnum].graphData.dispText.textHeight = 32;
	graphItems[itemnum].fgColor = color;
	graphItems[itemnum].bgColor = bgColor;
	graphItems[itemnum].type = TEXT;
	graphItems[itemnum].xpos = x;
	graphItems[itemnum].ypos = y;
  graphItems[itemnum].yend = y + graphItems[itemnum].graphData.dispText.textHeight;
	return itemnum;
}

/**
* @brief Add a polygon on LCD. May consist of up to 8 points. Points are connected in the order they are put into the array. A closing line from the last point to the first point is added automatically.
* @param itemnum: # of element to edit, -1 to add a new one
* @param x: X coord of first point on screen
* @param y: Y coord of first point on screen
* @param xpoints: Array with all X points. Coordinates relative to parameter x.
* @param ypoints: Array with all Y points. Coordinates relative to parameter y.
* @param numPoints: # of points (2-8)
* @param color: RGB 565 foreground color
* @retval 1: Element added; 0: error (all available graphics elemts occupied)

* @remark Unlike the GFXAddRect function, borders and transparency are currently not supported. If needed, draw 2 polygons within each other.
*/

int16_t GFXAddPoly(int16_t itemnum, uint16_t x, uint16_t y, int16_t* xpoints, int16_t* ypoints, uint8_t numPoints, uint16_t color) {
	if(itemnum == -1)
  {
    // Create new element
    if(uGfxUsed >= NUM_GFX_ELEMENTS) return -1;	//Abort if no further GFX elements are available
    itemnum = uGfxUsed;
    uGfxUsed++;
  }
	memcpy(graphItems[itemnum].graphData.dispPoly.px,xpoints,numPoints * 2);
	memcpy(graphItems[itemnum].graphData.dispPoly.py,ypoints,numPoints * 2);
	graphItems[itemnum].bgColor = numPoints;									//Use bgColor for # of points, since poly has no settable background color
	// Find highest element = lowest y coord
	int16_t lowestY = 32767;
	int16_t deltaY;
	int16_t highestY = -32767;
	for(uint8_t i=0;i<numPoints;i++) {
		if(ypoints[i] < lowestY) lowestY = ypoints[i];	//5
		if(ypoints[i] > highestY) highestY = ypoints[i];	//20
    if(((int16_t)x + xpoints[i]) > SCREEN_WIDTH) return 0; //Prevent exceeding screen width
	}
	deltaY = ypoints[0] - lowestY;	//15-5 = 10
	// Move all points in Y direction for the highest point to start at 0
	for(uint8_t i=0;i<numPoints;i++) {
		graphItems[itemnum].graphData.dispPoly.py[i] -= lowestY;	//-5
	}
	
	graphItems[itemnum].type = POLY;
	graphItems[itemnum].fgColor = color;
	graphItems[itemnum].xpos = x;
	graphItems[itemnum].ypos = y	- deltaY;
  graphItems[itemnum].yend = graphItems[itemnum].ypos + (highestY - lowestY) + 1; 	//Set maxY
	return itemnum;
}

/**
* @brief Add an arc (circle/annulus segment) on LCD.
* @param itemnum: # of element to edit, -1 to add a new one
* @param x: X coord of center point
* @param y: Y coord of center point
* @param radius: Outer radius of arc
* @param inRadius: Inner radius of arc
* @param angleStart: Start angle of arc, relative to a 3 o clock position in Q.15 float (16384 = 180�)
* @param angleEnd: End angle of arc, relative to a 3 o clock position in Q.15 float. Arc will be drawn counter-clockwise
* @param fillColor: RGB 565 foreground color
* @retval 1: Element added; 0: error (all available graphics elemts occupied)
*/
int16_t GFXAddArc(int16_t itemnum, uint16_t x, uint16_t y, uint16_t radius, uint16_t inRadius, int16_t angleStart, int16_t angleEnd, uint16_t fillColor) {
  if(itemnum == -1)
  {
    // Create new element
    if(uGfxUsed >= NUM_GFX_ELEMENTS) return -1;	//Abort if no further GFX elements are available
    itemnum = uGfxUsed;
    uGfxUsed++;
  }
	graphItems[itemnum].graphData.dispArc.radius = radius;
	graphItems[itemnum].graphData.dispArc.inRadius = inRadius;
	graphItems[itemnum].graphData.dispArc.startAngle = angleStart;
	graphItems[itemnum].graphData.dispArc.endAngle = angleEnd;
	graphItems[itemnum].fgColor = fillColor;
	graphItems[itemnum].type = ARC;
	graphItems[itemnum].xpos = x - radius;
	graphItems[itemnum].ypos = y - radius;
  graphItems[itemnum].yend = y + radius;
	//Calculate outer draw start/stop points
	//graphItems[itemnum].graphData.dispArc.px[0] = cos1(angleStart);
	graphItems[itemnum].graphData.dispArc.px[0] = (int16_t)(((int32_t)cos1(angleStart) * radius) / 32767L);
	graphItems[itemnum].graphData.dispArc.py[0] = (int16_t)(((int32_t)sin1(angleStart) * radius) / 32767L);
	graphItems[itemnum].graphData.dispArc.px[1] = (int16_t)(((int32_t)cos1(angleEnd) * radius) / 32767L);
	graphItems[itemnum].graphData.dispArc.py[1] = (int16_t)(((int32_t)sin1(angleEnd) * radius) / 32767L);
  graphItems[itemnum].graphData.dispArc.inrad2 = (int32_t)inRadius * (int32_t)inRadius;
  graphItems[itemnum].graphData.dispArc.rad2 = (int32_t)radius * (int32_t)radius;
	return itemnum;
}

/**
* @brief Add a rectangle for displaying
* @param itemnum: # of element to edit, -1 to add a new one
* @param x: x position of rectangle top left corner
* @param y: y position of rectangle top left corner
* @param width: Rectangle width in px
* @param height: Rectangle height in px
* @param fillColor: RGB 565 color for the inner surface of rectangle
* @param borderWidth: Rectangle border width in px. Set to 0 if rectangle shouldn't have a border
* @param borderColor: RGB 565 color for border
* @param insideTransparent: 0: fill inside of border with fillColor, 1: Area inside of border transparent
* @retval 1: Element added; 0: error (all available graphics elemts occupied)
*/
int16_t GFXAddRect(int16_t itemnum, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t fillColor, uint16_t borderWidth, uint16_t borderColor, uint8_t insideTransparent) {
	if(itemnum == -1)
  {
    // Create new element
    if(uGfxUsed >= NUM_GFX_ELEMENTS) return -1;	//Abort if no further GFX elements are available
    itemnum = uGfxUsed;
    uGfxUsed++;
  }
  if((x + width) > SCREEN_WIDTH) return -1; //Prevent exceeding screen width
	graphItems[itemnum].graphData.dispRect.width = width;
	graphItems[itemnum].graphData.dispRect.height = height;
	graphItems[itemnum].graphData.dispRect.borderWidth = borderWidth;
	graphItems[itemnum].graphData.dispRect.fillStyle = (insideTransparent & 1);
  if(((x & 1) == 0) && ((width & 1) == 0) && ((borderWidth & 1) == 0)) graphItems[itemnum].graphData.dispRect.fillStyle |= 8;
	graphItems[itemnum].fgColor = fillColor;
	graphItems[itemnum].bgColor = borderColor;
	graphItems[itemnum].type = RECT;
	graphItems[itemnum].xpos = x;
	graphItems[itemnum].ypos = y;
  graphItems[itemnum].yend = y + height;
	return itemnum;
}

/**
* @brief Add a monochrome/1bpp bitmap for displaying
* @param itemnum: # of element to edit, -1 to add a new one
* @param x: x position of bitmap top left corner
* @param y: y position of bitmap top left corner
* @param width: Bitmap width in px
* @param height: Bitmap height in px
* @param bmpData: Pointer to uint16_t array with bitmap data to display. Must be at least width*height bits in size
* @param fillColor: RGB 565 color for the set ('1') pixels
* @param bgColor: RGB 565 color for background ('0') pixels. Ignore if fillMode = 0
* @param bgTransparent: 0: fill background with bgColor, 1: Background transparent
* @retval 1: Element added; 0: error (all available graphics elemts occupied)
*/
int16_t GFXAddMonochromeBitmap(int16_t itemnum, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t* bmpData, uint16_t fillColor, uint16_t bgColor, uint8_t bgTransparent)
{
  if(itemnum == -1)
  {
    // Create new element
    if(uGfxUsed >= NUM_GFX_ELEMENTS) return -1;	//Abort if no further GFX elements are available
    itemnum = uGfxUsed;
    uGfxUsed++;
  }
  if((x + width) > SCREEN_WIDTH) return -1; //Prevent exceeding screen width
	graphItems[itemnum].graphData.dispRect.width = width;
	graphItems[itemnum].graphData.dispRect.height = height;
  graphItems[itemnum].graphData.dispRect.bitmap_data = bmpData;
	graphItems[itemnum].graphData.dispRect.fillStyle = 6 | (bgTransparent & 1); // Set bit 1 (bitmap) and 2 (1 bpp)
	graphItems[itemnum].fgColor = fillColor;
	graphItems[itemnum].bgColor = bgColor;
	graphItems[itemnum].type = RECT;
	graphItems[itemnum].xpos = x;
	graphItems[itemnum].ypos = y;
  graphItems[itemnum].yend = y + height;
	return itemnum;
}

/**
* @brief Add a 16 bit RGB 565 bitmap for displaying
* @param itemnum: # of element to edit, -1 to add a new one
* @param x: x position of bitmap top left corner
* @param y: y position of bitmap top left corner
* @param width: Bitmap width in px
* @param height: Bitmap height in px
* @param bmpData: Pointer to uint16_t array with bitmap data to display. Must be at least width*height uint16_t elements in size
* @param transpColor: RGB 565 color to be transparent if enableTransparency is 1
* @param enableTransparency: 1: Pixels are transparent if their color is transpColor
* @retval 1: Element added; 0: error (all available graphics elemts occupied)
*/
int16_t GFXAddColorBitmap(int16_t itemnum, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t* bmpData, uint16_t transpColor, uint8_t enableTransparency)
{
  if(itemnum == -1)
  {
    // Create new element
    if(uGfxUsed >= NUM_GFX_ELEMENTS) return -1;	//Abort if no further GFX elements are available
    itemnum = uGfxUsed;
    uGfxUsed++;
  }
  if((x + width) > SCREEN_WIDTH) return 0; //Prevent exceeding screen width
	graphItems[itemnum].graphData.dispRect.width = width;
	graphItems[itemnum].graphData.dispRect.height = height;
  graphItems[itemnum].graphData.dispRect.bitmap_data = bmpData;
	graphItems[itemnum].graphData.dispRect.fillStyle = 2 | (enableTransparency & 1); // Set bit 1 (bitmap)
  if(((x & 1) == 0) && ((width & 1) == 0)) graphItems[itemnum].graphData.dispRect.fillStyle |= 8;  // Enable 32 bit access on even X positions + widths
	graphItems[itemnum].bgColor = transpColor;
	graphItems[itemnum].type = RECT;
	graphItems[itemnum].xpos = x;
	graphItems[itemnum].ypos = y;
  graphItems[itemnum].yend = y + height;
	return itemnum;
}

/**
* @brief Add a circle on LCD.
* @param itemnum: # of element to edit, -1 to add a new one
* @param x: X coord of center point
* @param y: Y coord of center point
* @param radius: Outer radius circle
* @param fillColor: RGB 565 foreground color
* @param borderWidth: Circle border width in px. Set to 0 if circle shouldn't have a border
* @param borderColor: RGB 565 color for border
* @param insideTransparent: 0: fill inside of border with fillColor, 1: Area inside of border transparent
* @retval 1: Element added; 0: error (all available graphics elemts occupied)
*/
int16_t GFXAddCircle(int16_t itemnum, uint16_t x, uint16_t y, uint16_t radius, uint16_t fillColor, uint16_t borderWidth, uint16_t borderColor, uint8_t insideTransparent)
{
	if(itemnum == -1)
  {
    // Create new element
    if(uGfxUsed >= NUM_GFX_ELEMENTS) return -1;	//Abort if no further GFX elements are available
    itemnum = uGfxUsed;
    uGfxUsed++;
  }
  if((x + radius) > SCREEN_WIDTH) return -1; //Prevent exceeding screen width
	graphItems[itemnum].graphData.dispCircle.radius = radius;
	graphItems[itemnum].graphData.dispCircle.borderWidth = borderWidth;
	graphItems[itemnum].graphData.dispCircle.fillStyle = !insideTransparent;
  graphItems[itemnum].graphData.dispCircle.inrad2 = ((int32_t)radius - borderWidth) * ((int32_t)radius - borderWidth);
  graphItems[itemnum].graphData.dispCircle.rad2 = (int32_t)radius * (int32_t)radius;
	graphItems[itemnum].fgColor = fillColor;
	graphItems[itemnum].bgColor = borderColor;
	graphItems[itemnum].type = CIRCLE;
	graphItems[itemnum].xpos = x - radius;
	graphItems[itemnum].ypos = y - radius;
  graphItems[itemnum].yend = y + radius;
	return itemnum;
}

/**
* @brief Set the screen background color
* @param color: RGB 565 color
* @retval none
*/
void GFXSetBGColor(uint16_t color) {
	bgColor32 = ((uint32_t)color << 16) | color;
}

/**
* @brief Delete all graphics elements. Call this at the beginning of each frame before adding the elements. Screen won't be affected until GFXUpdateDisplay() is called.
* @retval none
*/
void GFXReset(void) {
	uGfxUsed = 0;       //Resetting the number of used elements to 0 is all that's needed
	#ifdef DMA_ENABLE
	bufindex = 0;
	#endif
}

/**
* @brief Update all GFX elements on the screen between startline and endline
* @param startline: First line to update [0...SCREEN_HEIGHT-2]
* @param endline: Last line to update [1...SCREEN_HEIGHT-1]
* @retval none
*/

void GFXUpdateDisplayRegion(uint16_t startline, uint16_t endline) {
  /* Select entire screen for updating */
  P_9481_SetUpdateLineRegion(startline, endline);
	/* Draw every line */
	for(uint16_t linenum=startline;linenum<=endline;linenum++) {
		// Reset line buffer with background color
    
    uint32_t* eraseaddr = (uint32_t*)(((uint16_t*)currline) + bufindex);
    uint32_t* endaddr = (uint32_t*)(((uint16_t*)currline) + bufindex + SCREEN_WIDTH);
		while(eraseaddr < endaddr) *eraseaddr++ = bgColor32;
    
		/* Check for every item if it lies on the line */
		for(int itemnum=0;itemnum<uGfxUsed;itemnum++) {
      // Get Y coord relative to item start
      if(GFXLineIsInItem(linenum, itemnum)) {
        //If item lies on line: Draw corresponding line
        uint16_t inty = linenum - graphItems[itemnum].ypos;
        switch(graphItems[itemnum].type)
        {
          case TEXT:
            GFXDrawTextLine(inty, itemnum);
            break;
          case RECT:
            GFXDrawRectLine(inty, itemnum);
            break;
          case POLY:
            GFXDrawPolyLine(inty, itemnum);
            break;
          case CIRCLE:
            GFXDrawCircleLine(inty, itemnum);
            break;
          case ARC:
            GFXDrawArcLine(inty, itemnum);
            break;
        }
      }
		}
		//Output the line on the screen
		P_9481_OutputScreenLine(linenum);
	}
  // Ensure the last line is output before returning
  //P_9481_WaitForDMACompletion();
}

/**
* @brief The main screen update function; renders + draws all graphics elements. Call in each frame after adding all elements.
* @retval none
*/
void GFXUpdateDisplay(void) {
  GFXUpdateDisplayRegion(0, SCREEN_HEIGHT - 1);
}

/**
* @brief Update the lines affected by a graphics elements. All other elements within these lines will also be updated.
* @retval none
*/
void GFXUpdateItem(int16_t itemnum) {
  GFXUpdateDisplayRegion(graphItems[itemnum].ypos, graphItems[itemnum].yend - 1);
}
//--------------------------------------------------------------
// Small delay
//--------------------------------------------------------------
void P_9481_Delay(volatile uint32_t nCount)
{
  while(nCount--)
  {
  }
}
/**
* @brief Convert a RGB color (0xRRGGBB) to the RGB 565 format for the display colors
* @parem rgb: Color to convert
* @retval RGB 565 color
*/
uint16_t RGBTo565(unsigned int rgb) {
	unsigned int r = (rgb & 0xFF) >> 3;
	unsigned int g = (rgb & 0xFF00) >> 10;
	unsigned int b = (rgb & 0xFF0000) >> 19;
	return(b << 11) | (g << 5) | r;
}

void print_fixed_str( uint32_t input, uint8_t length, uint8_t decimals, char* output, uint8_t spaces, uint8_t addNull)
/**
  * @brief  Convert integer to string as a fixed point number. Not directly related to LCD output
  * @param  input: input integer 
  * @param  length: Length of the string, including decimal point (1.23 has length 4)
  * @param  decimals: Digits after decimal point 
  * @param  outputString: array where output string is stored
  * @param  spaces: 0 - display leading zeroes; 1: replace leading zeroes with spaces
	* @param  addNull: 1 - add termating null character
  * @retval None.
*/
{
	uint8_t i, nonzero = 0;
	uint32_t maxDiv = 1;
	/* Calculate required divider to get to a one digit number */
	if(decimals == 0) {	//Depends on if there is a decimal point in the output
		for(i=0;i<length-1;i++) maxDiv *= 10;
	} else {
		for(i=0;i<length-2;i++) maxDiv *= 10;
	}
	//ERROR. Value to large
	if(input >= maxDiv * 10) {
		input = maxDiv * 10 - 1;
	}
		
	for(i=0;i<length;i++) {
		if((i == (length - decimals) - 1) && (decimals > 0)) {
			/* Decimal point reached */
			output[i] = '.';
		} else {
			/* Print digit */
			output[i] = (input / maxDiv) + 48;
			if(decimals == 0) {
					if((!nonzero) && spaces && (output[i] == '0') && (maxDiv > 1)) output[i] = ' ';
			} else {
				if((!nonzero) && spaces && (output[i] == '0') && (i < (length - decimals) - 2)) output[i] = ' ';
			}
			if(output[i] > 48) nonzero = 1;
			input %= maxDiv;
			maxDiv /= 10;
		}
	}
	if(addNull == 1) output[i] = 0;
}
