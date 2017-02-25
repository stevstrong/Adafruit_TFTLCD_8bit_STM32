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

#include <libmaple/gpio.h>

/*****************************************************************************/
// LCD controller chip identifiers
#define ID_932X    0
#define ID_7575    1
#define ID_9341    2
#define ID_HX8357D    3
#define ID_UNKNOWN 0xFF

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
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
/*****************************************************************************/
// Define pins and Output Data Registers
/*****************************************************************************/

#define TFT_DATA_PORT	GPIOB
// Port data bits D0..D7:
// enable only one from below lines corresponding to your HW setup:
#define TFT_DATA_LOW_NIBBLE	1 // take the lower 8 bits: 0..7
//#define TFT_DATA_HIGH_NIBBLE	1 // take the higher 8 bits: 8..15

//Control pins |RD |WR |RS |CS |RST|
#define TFT_CNTRL_PORT	GPIOA
#define TFT_RD			PA0
#define TFT_WR			PA1
#define TFT_RS			PA2
#define TFT_CS			PA3
#define TFT_RST			PB10 //PB0

#define TFT_RD_MASK		BIT0 // digitalPinToBitMask(TFT_RD) // 
#define TFT_WR_MASK		BIT1 // digitalPinToBitMask(TFT_WR) // 
#define TFT_RS_MASK		BIT2 // digitalPinToBitMask(TFT_RS) // 
#define TFT_CS_MASK		BIT3 // digitalPinToBitMask(TFT_CS) // 

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
	#define CS_ACTIVE_CD_COMMAND	{ CS_ACTIVE; CD_COMMAND; }
#else
	// use fast bit toggling, very fast speed!
extern gpio_reg_map * cntrlRegs;
	#define RD_ACTIVE				{ cntrlRegs->BRR  = TFT_RD_MASK; }
	#define RD_IDLE					{ cntrlRegs->BSRR = TFT_RD_MASK; }
	#define WR_ACTIVE				{ cntrlRegs->BRR  = TFT_WR_MASK; }
	#define WR_IDLE					{ cntrlRegs->BSRR = TFT_WR_MASK; }
	#define CD_COMMAND				{ cntrlRegs->BRR  = TFT_RS_MASK; }
	#define CD_DATA					{ cntrlRegs->BSRR = TFT_RS_MASK; }
	#define CS_ACTIVE				{ cntrlRegs->BRR  = TFT_CS_MASK; }
	#define CS_IDLE					{ cntrlRegs->BSRR = TFT_CS_MASK; }
	#define CS_ACTIVE_CD_COMMAND	{ cntrlRegs->BRR  = (TFT_CS_MASK|TFT_RS_MASK); }
#endif

#define WR_STROBE { WR_ACTIVE; WR_IDLE; }

extern uint8_t read8_(void);
#define read8(x) ( x = read8_() )

extern gpio_reg_map * dataRegs;

#if defined(TFT_DATA_LOW_NIBBLE)
  //#warning "Using lower data nibble..."
	// set the pins to input mode
	#define setReadDir() ( dataRegs->CRL = 0x88888888 )	// set the lower 8 bits as input
	//#define setReadDir() ( dataRegs->CRL = 0x44444444 )	// set the lower 8 bits as input floating
	// set the pins to output mode
	#define setWriteDir() ( dataRegs->CRL = 0x33333333 )	// set the lower 8 bits as output
	#define TFT_DATA_SHIFT 0
#elif defined(TFT_DATA_HIGH_NIBBLE)
  #warning "Using high data nibble..."
	// set the pins to input mode
	#define setReadDir() ( dataRegs->CRH = 0x88888888 )	// set the upper 8 bits as input
	// set the pins to output mode
	#define setWriteDir() ( dataRegs->CRH = 0x33333333 )	// set the lower 8 bits as output
	#define TFT_DATA_SHIFT 8
#endif

// set pins to output the 8 bit value
#if 0 // slow write
 #define write8(c) { Serial.print(" write8: "); Serial.print(c,HEX); Serial.write(','); \
					digitalWrite(PB0, (c&BIT0)?HIGH:LOW); \
					digitalWrite(PB1, (c&BIT1)?HIGH:LOW); \
					digitalWrite(PB2, (c&BIT2)?HIGH:LOW); \
					digitalWrite(PB3, (c&BIT3)?HIGH:LOW); \
					digitalWrite(PB4, (c&BIT4)?HIGH:LOW); \
					digitalWrite(PB5, (c&BIT5)?HIGH:LOW); \
					digitalWrite(PB6, (c&BIT6)?HIGH:LOW); \
					digitalWrite(PB7, (c&BIT7)?HIGH:LOW); \
					WR_STROBE; }
#else
 #define write8(c) { uint32_t val = (((c^0x00FF)<<16) | c)<<TFT_DATA_SHIFT; \
					/*Serial.print(" write8: "); Serial.print(val,HEX); Serial.write(',');*/ \
					dataRegs->BSRR = val; WR_STROBE; }
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
  void     setAddrWindow(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
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
extern uint32_t readReg32(uint8_t r);
extern void writeCommand(uint16_t c);
extern void writeRegister8(uint16_t a, uint8_t d);
extern void writeRegister16(uint16_t a, uint16_t d);
extern void writeRegister24(uint16_t a, uint32_t d);
extern void writeRegister32(uint16_t a, uint32_t d);
extern void writeRegisterPair(uint16_t aH, uint16_t aL, uint16_t d);

extern Adafruit_TFTLCD_8bit_STM32 TFT;

#endif
