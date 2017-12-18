// IMPORTANT: LIBRARY MUST BE SPECIFICALLY CONFIGURED FOR EITHER TFT SHIELD
// OR BREAKOUT BOARD USAGE.  SEE RELEVANT COMMENTS IN Adafruit_TFTLCD.h

// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

/*

Graphics-test Benchmark  Time (microseconds)
--------------------------------------------
Screen fill              62999
Text                     20483
Lines                    173830
Horiz/Vert Lines         7604
Rectangles (outline)     5958
Rectangles (filled)      148717
Circles (filled)         91783
Circles (outline)        75096
Triangles (outline)      55138
Triangles (filled)       90553
Rounded rects (outline)  27257
Rounded rects (filled)   186639
*/

#include "Adafruit_TFTLCD_8bit_STM32.h"
#include "ili9341.h"

/**/
#define TFTLCD_DELAY8 0xFF

const uint8_t ILI9341_regValues_ada[] = {        // Adafruit_TFTLCD only works with EXTC=0
	//                     0xF6, 3, 0x00, 0x01, 0x00,  //Interface Control needs EXTC=1 TM=0, RIM=0
	//            0xF6, 3, 0x01, 0x01, 0x03,  //Interface Control needs EXTC=1 RM=1, RIM=1
/**/
	0xF6, 3, 0x09, 0x01, 0x03,  //Interface Control needs EXTC=1 RM=0, RIM=1
	0xB0, 1, 0x40,      //RGB Signal [40] RCM=2
	0xB4, 1, 0x00,      //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
	0xC0, 1, 0x23,      //Power Control 1 [26]
	0xC1, 1, 0x10,      //Power Control 2 [00]
	0xC5, 2, 0x2B, 0x2B,        //VCOM 1 [31 3C]
	0xC7, 1, 0xC0,      //VCOM 2 [C0]
	0x36, 1, 0x48,      //Memory Access [00]
	0xB1, 2, 0x00, 0x1B,        //Frame Control [00 1B]
	0xB7, 1, 0x07,      //Entry Mode [00]
//	TFTLCD_DELAY8, 1,
};

/*
.write16: 36_write8: 0__write8: 36__write8: 48_
.write16: 33_write8: 0__write8: 33__write8: 0__write8: 0__write8: 1__write8: 40__write8: 0__write8: 0_
.write16: 37_write8: 0__write8: 37__write8: 0__write8: 0_
.write16: 13_write8: 0__write8: 13_
.write16: 21_write8: 0__write8: 21_
*/
const uint8_t ILI9341_regValues_post[] =
{ // post-init settings, sniffed from David's lib
	0x36, 1, 0x48,      //Memory Access [00]
	0x33, 6, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00,
	0x37, 2, 0x00, 0x00,
	0x13, 0,			// normaldisp
	ILI9341_INVERTOFF, 0,			// invert off
};
/*****************************************************************************/
static void WriteCmdParamN(uint16_t cmd, int8_t N, const uint8_t * block)
{
	//Serial.print("cmd: "); Serial.print(cmd,HEX);
    writeCommand(cmd);
	//Serial.print(", data: ");
	while (N-- > 0) {
		uint8_t u8 = *block++;
		CD_DATA;
		//Serial.print(u8,HEX);
		write8(u8);
	}
    CS_IDLE;
	//Serial.write('\n');
}

/*****************************************************************************/
static void init_table(const uint8_t *table, int16_t size)
{
	while (size > 0) {
		uint8_t cmd = *table++;
		uint8_t len = *table++;
		if (cmd == TFTLCD_DELAY8) {
			//Serial.print("delay: "); Serial.println(len);
			delay(len);
			len = 0;
		} else {
			WriteCmdParamN(cmd, len, table);
			table += len;
		}
		size -= len + 2;
	}
}

const uint8_t reset_off[] = {
	0x01, 0,            //Soft Reset
	TFTLCD_DELAY8, 150,  // .kbv will power up with ONLY reset, sleep out, display on
	0x28, 0,            //Display Off
	0x3A, 1, 0x55,      //Pixel read=565, write=565.
};
const uint8_t wake_on[] = {
	0x11, 0,            //Sleep Out
	TFTLCD_DELAY8, 150,
	0x29, 0,            //Display On
	//additional settings
	ILI9341_INVERTOFF, 0,			// invert off
	0x36, 1, 0x48,      //Memory Access
	0xB0, 1, 0x40,      //RGB Signal [40] RCM=2
};
/*****************************************************************************/
void ili9341_begin(void)
{
//	Serial.println("\nILI9341 begin...");
	init_table(reset_off, sizeof(reset_off));
	//init_table(ILI9341_regValues_ada, sizeof(ILI9341_regValues_ada));   //can change PIXFMT
	init_table(wake_on, sizeof(wake_on));
	//init_table(ILI9341_regValues_post, sizeof(ILI9341_regValues_post));
}

/*****************************************************************************/
// Sets the LCD address window. Relevant to all write routines.
// Input coordinates are assumed pre-sorted (e.g. x2 >= x1).
/*****************************************************************************/
void ili9341_setAddrWindow(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	writeRegister32(ILI9341_COLADDRSET, ((uint32_t)(x1<<16) | x2));  // HX8357D uses same registers!
	writeRegister32(ILI9341_PAGEADDRSET, ((uint32_t)(y1<<16) | y2)); // HX8357D uses same registers!
}

/*****************************************************************************/
void ili9341_fillScreen(uint16_t color)
{
}

/*****************************************************************************/
void ili9341_drawPixel(int16_t x, int16_t y, uint16_t color)
{
}

/*****************************************************************************/
void ili9341_setRotation(uint8_t x)
{
}

/*****************************************************************************/
uint16_t ili9341_readPixel(int16_t x, int16_t y)
{
	return 0;
}
