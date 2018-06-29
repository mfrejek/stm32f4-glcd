#include <string.h>
#include <stdint.h>

// Set this define to enable DMA for writing to the display. 
// Normally, there is no reason for not enabling it (DMA makes the display update way faster); except if the HAL doesn't support DMA
#define DMA_ENABLE

//--------------------------------------------------------------
// FSMC addresses
// Bank   = Bank-1 / PSRAM-1        => BaseAdr 0x60000000
// RS-Pin = PE3=FSMC_A18 = BitNr 18 => Offset 2^(18+1)=0x00080000 => Write to addr 0x60080000 for A18 to bet set while writing
// (see pages 1316+1317 of reference manual)
//--------------------------------------------------------------
#define LCD_REG_ADR   0x60000000                              // FSMC base address
#define LCD_RAM_ADR   0x60080000                              // RAM address with FSMC_A18 (connected to RS) set
#define LCD_REG       (*((volatile uint16_t *) LCD_REG_ADR))  // RS = 0
#define LCD_RAM       (*((volatile uint16_t *) LCD_RAM_ADR))  // RS = 1

#define SCREEN_WIDTH	    320       // Screen width in px. Must be an even number
#define SCREEN_HEIGHT	    480       // Screen height in px. Must be an even number
#define NUM_GFX_ELEMENTS	128       // Maximum number of graphics elements which can be used
#define MAX_POLY_POINTS   	8       // Maximum number of polygon points. Please leave at 8

#ifdef DMA_ENABLE
  #define CURRLINE_LEN SCREEN_WIDTH * 2
	extern volatile uint16_t bufindex;
#else
  #define CURRLINE_LEN SCREEN_WIDTH
	#define bufindex 0
#endif

extern uint16_t currline[CURRLINE_LEN];
extern uint32_t bgColor32;	//Background color
/*
 * Fast sine/cosine calculation functions
 * Source: https://www.atwillys.de/content/cc/sine-lookup-for-embedded-in-c/?lang=en
 * The number of bits of our data type: here 16 (sizeof operator returns bytes).
 */
#define INT16_BITS  (8 * sizeof(int16_t))
#ifndef INT16_MAX
#define INT16_MAX   ((1<<(INT16_BITS-1))-1)
#endif
 

// Some color defines
#define COLOR_BLUE          0xF800
#define COLOR_GREEN         0x07E0
#define COLOR_RED           0x001F
#define COLOR_CYAN          0xFFE0
#define COLOR_MAGENTA       0xF81F
#define COLOR_YELLOW        0x07FF
#define COLOR_LIGHTBLUE     0xFC10
#define COLOR_LIGHTGREEN    0x87F0
#define COLOR_LIGHTRED      0x841F
#define COLOR_LIGHTCYAN     0xFFF0
#define COLOR_LIGHTMAGENTA  0xFC1F
#define COLOR_LIGHTYELLOW   0x87FF
#define COLOR_DARKBLUE      0x8000
#define COLOR_DARKGREEN     0x0400
#define COLOR_DARKRED       0x0010
#define COLOR_DARKCYAN      0x8400
#define COLOR_DARKMAGENTA   0x8010
#define COLOR_DARKYELLOW    0x0410
#define COLOR_WHITE         0xFFFF
#define COLOR_LIGHTGRAY     0xD69A
#define COLOR_GRAY          0x8410
#define COLOR_DARKGRAY      0x4208
#define COLOR_BLACK         0x0000
#define COLOR_BROWN         0x2954
#define COLOR_ORANGE        0x053F

#define BG_TRANSPARENT    0
#define BG_OPAQUE         1
/*
 * "5 bit" large table = 32 values. The mask: all bit belonging to the table
 * are 1, the all above 0.
 */
#define TABLE_BITS  (5)
#define TABLE_SIZE  (1<<TABLE_BITS)
#define TABLE_MASK  (TABLE_SIZE-1)

#define BINARY_MID (TABLE_SIZE / 2)
#define BINARY_INC (BINARY_MID / 2)
 
/*
 * The lookup table is to 90DEG, the input can be -360 to 360 DEG, where negative
 * values are transformed to positive before further processing. We need two
 * additional bits (*4) to represent 360 DEG:
 */
#define LOOKUP_BITS (TABLE_BITS+2)
#define LOOKUP_MASK ((1<<LOOKUP_BITS)-1)
#define FLIP_BIT    (1<<TABLE_BITS)
#define NEGATE_BIT  (1<<(TABLE_BITS+1))
#define INTERP_BITS (INT16_BITS-1-LOOKUP_BITS)
#define INTERP_MASK ((1<<INTERP_BITS)-1)
 

enum GFXType {TEXT, RECT, POLY, CIRCLE, ARC};
enum FontSize {XSMALL, SMALL, MEDIUM, LARGE};

struct gfxRect {					//Struct for rectangle and bitmap
  uint16_t* bitmap_data; //Pointer to bitmap data; if type is bitmap. For rectangle, this pointer can be invalid
	uint16_t width;
	uint16_t height;
	uint16_t borderWidth;
	uint8_t fillStyle;			//Bit 0: 1 = fill with foreground color, 0 = do not fill; Bit 1: 0 = rectangle, 1 = bitmap; Bit 2: 1 = 16bpp, 0 = 1bpp; Bit 3: 1 = 32 bit access pssible for speed, 0 = 16 bit access needed
  // If bitmap is 1 bpp, the foreground/background colors are defined by the fgColor/bgColor
	uint8_t rfu;
};

struct gfxText {					    //Struct for text
	unsigned char drawText[33];	//Max. 33 chars per line
	uint8_t length;							//For higher performance
	uint8_t drawMode; 				  //Mostly RFU, currently: 1 = Background transparent
	uint8_t textHeight;					//Height in pixels
};

struct gfxPoly {
	int16_t px[MAX_POLY_POINTS];								//X coords of polygon points
	int16_t py[MAX_POLY_POINTS];								//Y coords of polygon points
};

struct gfxCircle {					//Struct for circle
  int32_t rad2;             //Square of radius
  int32_t inrad2;           //Square of radius inside border
	uint16_t radius;
	uint16_t borderWidth;
  
	uint8_t fillStyle;			//1 = fill with foreground color, 0 = do not fill
	uint8_t rfu;
};

struct gfxArc {					//Struct for arc
  int32_t rad2;             //Square of radius
  int32_t inrad2;           //Square of radius inside
	uint16_t radius;
	uint16_t inRadius;
	int16_t startAngle;
	int16_t endAngle;
	int16_t px[2];								//X coords of corner points
	int16_t py[2];								//Y coords of corner points
	uint8_t fillStyle;			//1 = fill with foreground color, 0 = do not fill
	uint8_t rfu;
};

union gfxData {
	struct gfxText dispText;
	struct gfxRect dispRect;
	struct gfxPoly dispPoly;
	struct gfxCircle dispCircle;
	struct gfxArc dispArc;
};

struct gfxItem {
	union gfxData graphData;
	uint16_t xpos;			        //X position of upper left corner
	uint16_t ypos;			        //Y position of upper left corner
	uint16_t fgColor;		        //Foreground color
	uint16_t bgColor;		        //Background color
  uint16_t yend;              //Y pos pf lower left corner
	enum GFXType type;				  //Graphics element type
	uint8_t rfu;							  //To fill word
	uint16_t rfu2;						  //To fill word
};

struct sortItem {
	int16_t xpos;
	int16_t tangent;
};

void P_9481_HardwareInit(void);
void P_9481_DisplayInit(void);
void P_9481_SetUpdateLineRegion(uint16_t startline, uint16_t endline);
void P_9481_OutputScreenLine(uint16_t linenum);
void P_9481_WaitForDMACompletion(void);

void P_9481_Delay(volatile uint32_t nCount);


void P_9481_TestVecfont(void);
uint16_t RGBTo565(unsigned int rgb);

void GFXReset(void);
void GFXSetBGColor(uint16_t color);
void GFXUpdateDisplay(void);
void GFXUpdateItem(int16_t itemnum);

int16_t GFXAddText(int16_t itemnum, char* text, uint16_t x, uint16_t y, uint16_t color, enum FontSize size, uint8_t bgTransparent, uint16_t bgColor);
int16_t GFXAddPoly(int16_t itemnum, uint16_t x, uint16_t y,int16_t* xpoints, int16_t* ypoints, uint8_t numPoints, uint16_t color);
int16_t GFXAddRect(int16_t itemnum, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t fillColor, uint16_t borderWidth, uint16_t borderColor, uint8_t insideTransparent);
int16_t GFXAddCircle(int16_t itemnum, uint16_t x, uint16_t y, uint16_t radius, uint16_t fillColor, uint16_t borderWidth, uint16_t borderColor, uint8_t insideTransparent);
int16_t GFXAddArc(int16_t itemnum, uint16_t x, uint16_t y, uint16_t radius, uint16_t inRadius, int16_t angleStart, int16_t angleEnd, uint16_t fillColor);
int16_t GFXAddColorBitmap(int16_t itemnum, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t* bmpData, uint16_t transpColor, uint8_t enableTransparency);
int16_t GFXAddMonochromeBitmap(int16_t itemnum, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t* bmpData, uint16_t fillColor, uint16_t bgColor, uint8_t bgTransparent);
void print_fixed_str( uint32_t input, uint8_t length, uint8_t decimals, char* output, uint8_t spaces, uint8_t addNull);
