// See SetupX_Template.h for all options available
#define USER_SETUP_ID 500

#define GC9A01_DRIVER
#define TFT_PARALLEL_8_BIT

#define TFT_CS    2
#define TFT_DC   18  // "RS" Data Command control pin - must use a GPIO in the range 0-31
#define TFT_RST  21

#define TFT_WR    3  // Write strobe control pin - must use a GPIO in the range 0-31
#define TFT_RD    -1 // Read pin not connected

#define TFT_D0   10  // Must use GPIO in the range 0-31 for the data bus
#define TFT_D1   11  // so a single register write sets/clears all bits
#define TFT_D2   12
#define TFT_D3   13
#define TFT_D4   14
#define TFT_D5   15
#define TFT_D6   16
#define TFT_D7   17

#define TFT_BL 42 //BL is on MTMS physical pin 48
#define TFT_BACKLIGHT_ON HIGH

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:.
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
#define SMOOTH_FONT

#define SPI_FREQUENCY  40000000

#define SPI_READ_FREQUENCY  20000000

#define SPI_TOUCH_FREQUENCY  2500000

// #define SUPPORT_TRANSACTIONS