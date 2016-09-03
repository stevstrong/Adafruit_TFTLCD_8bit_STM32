// Paint example specifically for the TFTLCD breakout board.
// If using the Arduino shield, use the tftpaint_shield.pde sketch instead!
// DOES NOT CURRENTLY WORK ON ARDUINO LEONARDO

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD_8bit_STM32.h> // Hardware-specific library
#include <TouchScreen_STM.h>


// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define LED_PIN PC13

Adafruit_TFTLCD_8bit_STM32 tft;

#define BOXSIZE 40
#define PENRADIUS 3
int oldcolor, currentcolor;
/*
#define TFT_RD         PA0
#define TFT_WR         PA1
#define TFT_RS         PA2
#define TFT_CS         PA3
*/
// overlaping:
#define XM TFT_RS // 330 Ohm // must be an analog pin !!!
#define YP TFT_CS // 500 Ohm // must be an analog pin !!!
#define XP PB0 //TFT_D0 // 330 Ohm // can be a digital pin
#define YM PB1 //TFT_D1 // 500 Ohm // can be a digital pin

TouchScreen ts = TouchScreen(XP, YP, XM, YM);

char str[250];	// used for sprintf
/*****************************************************************************/
void setup(void)
{
	Serial.begin(115200);
	delay(6000); // allow time for OS to enumerate USB as COM port
	Serial.print("\n*** Paint demo with easy touch calibration process ***\n");

	//Serial.println("Resetting the screen...");
	uint16_t identifier = 0;
do {
	tft.reset();
  identifier = tft.readID();
  if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
 /*   Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
*/
    identifier = 0;
  }
  //delay(250);
}  while (identifier == 0);

  tft.begin(identifier);
  
	ts.rangeSet(TFTWIDTH, TFTHEIGHT);	// set display X and Y resolution of the display screen
							// the returned coordinates will be automatically mapped to these ranges

// Calibration
// needs sequentially touching two, diagonally opposite corners (check serial output!)
// the touching can include small circular movements around the corners
// it will keep calibrating the corner as long as pressure is applied
  tft.fillScreen(WHITE);
  tft.setCursor(0, 30);
  tft.setTextColor(BLACK);  tft.setTextSize(1);
  tft.println("Calibration");
  tft.println("Press one corner...");
	Serial.println("Calibrating the touch surface");
	Serial.print("> please press one corner...");
	ts.calibratePoint();
  tft.println("Now press the opposite corner...");
	Serial.print("ok.\n> and now press the diagonally opposite corner...");
	ts.calibratePoint();
  tft.println("Calibration done.");
	Serial.println("ok.\nCalibration done.");
  delay(1000);
  tft.fillScreen(BLACK);
  //tft.setRotation(2);

  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
  tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, GREEN);
  tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, CYAN);
  tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, BLUE);
  tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, MAGENTA);
  // tft.fillRect(BOXSIZE*6, 0, BOXSIZE, BOXSIZE, WHITE);
 
  tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
  currentcolor = RED;
 
  pinMode(LED_PIN, OUTPUT);
}

#define MINPRESSURE 10
#define MAXPRESSURE 1000

void loop()
{
	TSPoint p;	// a point object holds x, y and z coordinates
/*
  digitalWrite(LED_PIN, HIGH);
  TSPoint p = ts.getPoint();
  digitalWrite(LED_PIN, LOW);
*/

  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  
  if ( ts.getPoint(&p) ) { //  returns 'true' if touch is detected, otherwise 'false'
		// the coordinates will be mapped to the set display ranges by using 'rangeSet()' in setup
		// x, y (in pixels) can be directly used for display routines!
		//sprintf(str,"X = %4i, Y = %4i, P = %4i\n", p.x, p.y, p.z);	Serial.print(str);
    /*
    if (p.y < (TS_MINY-5)) {
      Serial.println("erase");
      // press the bottom of the screen to erase 
      tft.fillRect(0, BOXSIZE, tft.width(), tft.height()-BOXSIZE, BLACK);
    }*/
    if (p.y < BOXSIZE) {
       oldcolor = currentcolor;

       if (p.x < BOXSIZE) { 
         currentcolor = RED; 
         tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
       } else if (p.x < BOXSIZE*2) {
         currentcolor = YELLOW;
         tft.drawRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, WHITE);
       } else if (p.x < BOXSIZE*3) {
         currentcolor = GREEN;
         tft.drawRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, WHITE);
       } else if (p.x < BOXSIZE*4) {
         currentcolor = CYAN;
         tft.drawRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, WHITE);
       } else if (p.x < BOXSIZE*5) {
         currentcolor = BLUE;
         tft.drawRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, WHITE);
       } else if (p.x < BOXSIZE*6) {
         currentcolor = MAGENTA;
         tft.drawRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, WHITE);
       }

       if (oldcolor != currentcolor) {
          if (oldcolor == RED) tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
          if (oldcolor == YELLOW) tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
          if (oldcolor == GREEN) tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, GREEN);
          if (oldcolor == CYAN) tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, CYAN);
          if (oldcolor == BLUE) tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, BLUE);
          if (oldcolor == MAGENTA) tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, MAGENTA);
       }
    }
    if (((p.y-PENRADIUS) > BOXSIZE) && ((p.y+PENRADIUS) < tft.height())) {
      tft.fillCircle(p.x, p.y, PENRADIUS, currentcolor);
    }
  }
}

