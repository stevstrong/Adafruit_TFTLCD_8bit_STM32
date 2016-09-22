// IMPORTANT: SEE COMMENTS @ LINE 15 REGARDING SHIELD VS BREAKOUT BOARD USAGE.

// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

#ifndef _ADAFRUIT_TFTLCD_8BIT_STM32_H_
#define _ADAFRUIT_TFTLCD_8BIT_STM32_H_

    #define PROGMEM
    #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
    #define pgm_read_word(addr) (*(const unsigned short *)(addr))

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_GFX.h>

/*****************************************************************************/
#define TFTWIDTH   240
#define TFTHEIGHT  320

// Initialization command tables for different LCD controllers
#define TFTLCD_DELAY 0xFF

// For compatibility with sketches written for older versions of library.
// Color function name was changed to 'color565' for parity with 2.2" LCD
// library.
#define Color565 color565

/*****************************************************************************/
// Define pins and Output Data Registers
/*****************************************************************************/

#define TFT_DATA       GPIOB
// Port data bits D0..D7:
// enable only one from below lines corresponding to your HW setup:
#define TFT_DATA_NIBBLE	0 // take the lower 8 bits: 0..7
//#define TFT_DATA_NIBBLE	8 // take the higher 8 bits: 8..15

//Control pins |RD |WR |RS |CS |RST|
#define TFT_CNTRL      GPIOA
#define TFT_RD         PA0
#define TFT_WR         PA1
#define TFT_RS         PA2
#define TFT_CS         PA3
#define TFT_RST        PB8 //PB0

#define TFT_RD_MASK    BIT0 // digitalPinToBitMask(TFT_RD) // 
#define TFT_WR_MASK    BIT1 // digitalPinToBitMask(TFT_WR) // 
#define TFT_RS_MASK    BIT2 // digitalPinToBitMask(TFT_RS) // 
#define TFT_CS_MASK    BIT3 // digitalPinToBitMask(TFT_CS) // 

#if 0
	// use old definition, standard bit toggling, low speed
	#define RD_ACTIVE    digitalWrite(TFT_RD, LOW)
	#define RD_IDLE      digitalWrite(TFT_RD, HIGH)
	#define WR_ACTIVE    digitalWrite(TFT_WR, LOW)
	#define WR_IDLE      digitalWrite(TFT_WR, HIGH)
	#define CD_COMMAND   digitalWrite(TFT_RS, LOW)
	#define CD_DATA      digitalWrite(TFT_RS, HIGH)
	#define CS_ACTIVE    digitalWrite(TFT_CS, LOW)
	#define CS_IDLE      digitalWrite(TFT_CS, HIGH)
#else
	// use fast bit toggling, very fast speed!
	#define RD_ACTIVE    { TFT_CNTRL->regs->BRR  = TFT_RD_MASK; }
	#define RD_IDLE      { TFT_CNTRL->regs->BSRR = TFT_RD_MASK; }
	#define WR_ACTIVE    { TFT_CNTRL->regs->BRR  = TFT_WR_MASK; }
	#define WR_IDLE      { TFT_CNTRL->regs->BSRR = TFT_WR_MASK; }
	#define CD_COMMAND   { TFT_CNTRL->regs->BRR  = TFT_RS_MASK; }
	#define CD_DATA      { TFT_CNTRL->regs->BSRR = TFT_RS_MASK; }
	#define CS_ACTIVE    { TFT_CNTRL->regs->BRR  = TFT_CS_MASK; }
	#define CS_IDLE      { TFT_CNTRL->regs->BSRR = TFT_CS_MASK; }
#endif

#define WR_STROBE { WR_ACTIVE; WR_IDLE; }

//Set pins to the 8 bit number
#define write8(c) { TFT_DATA->regs->BSRR = (((c^0xFF)<<16) | (c))<<TFT_DATA_NIBBLE; WR_STROBE; }

extern uint8_t read8_(void);
#define read8(x) ( x = read8_() )

// set the pins to output mode
// not required to mask and assign, because all pins of bus are set together
//each pin is configured by four bits, and 0b0011 or 0x3 means output mode (same as pinmode())
#if TFT_DATA_NIBBLE>0
	#define setWriteDir() ( TFT_DATA->regs->CRH = 0x33333333 )	// set the lower 8 bits as output
#else
	#define setWriteDir() ( TFT_DATA->regs->CRL = 0x33333333 )	// set the lower 8 bits as output
#endif

// set the pins to input mode
// not required to mask and assign, because all pins of bus are set together
// 8 in hex is 0b1000, which means input, same as pinmode()
#if TFT_DATA_NIBBLE>0
  #define setReadDir() ( TFT_DATA->regs->CRH = 0x88888888 )	// set the upper 8 bits as input
#else
  #define setReadDir() ( TFT_DATA->regs->CRL = 0x88888888 )	// set the lower 8 bits as input
#endif


/*****************************************************************************/

#define swap(a, b) { int16_t t = a; a = b; b = t; }

/*****************************************************************************/
// **** IF USING THE LCD BREAKOUT BOARD, COMMENT OUT THIS NEXT LINE. ****
// **** IF USING THE LCD SHIELD, LEAVE THE LINE ENABLED:             ****

//#define USE_ADAFRUIT_SHIELD_PINOUT 1

/*****************************************************************************/
class Adafruit_TFTLCD_8bit_STM32 : public Adafruit_GFX {

 public:

  //Adafruit_TFTLCD_8bit_STM32(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t rst);
  Adafruit_TFTLCD_8bit_STM32(void);

  void     begin(uint16_t id = 0x9328);
  void     drawPixel(int16_t x, int16_t y, uint16_t color);
  void     drawFastHLine(int16_t x0, int16_t y0, int16_t w, uint16_t color);
  void     drawFastVLine(int16_t x0, int16_t y0, int16_t h, uint16_t color);
  void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
  void     fillScreen(uint16_t color);
  void     reset(void);
  void     setRegisters8(uint8_t *ptr, uint8_t n);
  void     setRegisters16(uint16_t *ptr, uint8_t n);
  void     setRotation(uint8_t x);
       // These methods are public in order for BMP examples to work:
  void     setAddrWindow(int x1, int y1, int x2, int y2);
  void     invertDisplay(boolean i),
			pushColors(uint16_t *data, int16_t len, boolean first),
           drawBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t * bitmap);

  uint16_t readPixel(int16_t x, int16_t y),
           readID(void);

 private:

  void     init(),
           // These items may have previously been defined as macros
           // in pin_magic.h.  If not, function versions are declared:
           //setLR(void),
           flood(uint16_t color, uint32_t len);
  uint8_t  driver;

};

extern uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
extern uint16_t readReg(uint8_t r);
//extern void writeCommand(uint16_t c);
extern void writeRegister8(uint8_t a, uint8_t d);
extern void writeRegister16(uint16_t a, uint16_t d);
extern void writeRegister24(uint8_t a, uint32_t d);
extern void writeRegister32(uint8_t a, uint32_t d);
extern void writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d);


#endif
