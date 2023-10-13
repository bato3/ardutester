/* ************************************************************************
 *
 *   ATmega324/664/1284 specific global configuration, setup and settings
 *
 *   (c) 2012-2016 by Markus Reschke
 *   based on code from Markus Frejek and Karl-Heinz K�bbeler
 *
 * ************************************************************************ */


/* source management */
#define CONFIG_664_H



/* ************************************************************************
 *   LCD module
 * ************************************************************************ */


/*
 *  LCD module / controller
 *
 *  Please uncomment the package matching your LCD module
 *  and adjust settings.
 *
 *  To uncomment, remove the enclosing "#if 0" and "#endif" or
 *  put a "//" in front of both.
 */


/*
 *  HD44780, 4 bit parallel
 *  - if you change LCD_DB4/5/6/7 comment out LCD_DB_STD!
 */

#if 0
#define LCD_HD44780_PAR4
#define LCD_TEXT                        /* character display */
#define LCD_PORT         PORTD          /* port data register */
#define LCD_DDR          DDRD           /* port data direction register */
#define LCD_DB_STD                      /* use standard pins 0-3 for DB4-7 */
#define LCD_DB4          PD0            /* port pin used for DB4 */
#define LCD_DB5          PD1            /* port pin used for DB5 */
#define LCD_DB6          PD2            /* port pin used for DB6 */
#define LCD_DB7          PD3            /* port pin used for DB7 */
#define LCD_RS           PD4            /* port pin used for RS */
#define LCD_EN1          PD5            /* port pin used for E */
#define LCD_CHAR_X       16             /* characters per line */
#define LCD_CHAR_Y       2              /* number of lines */
#define FONT_HD44780_INT                /* internal 5x7 font, international */
#endif


/*
 *  ST7565R, SPI interface
 *  - settings for Electronic Assembly EA DOGM/DOGL128-6
 *  - uses LCD_CS to support rotary encoder in parallel at PD2/3
 */

//#if 0
#define LCD_ST7565R_SPI
#define LCD_GRAPHIC                     /* monochrome graphic display */
#define LCD_PORT         PORTD          /* port data register */
#define LCD_DDR          DDRD           /* port data direction register */
#define LCD_RESET        PD0            /* port pin used for /RES */
#define LCD_A0           PD1            /* port pin used for A0 */
#define LCD_SCL          PD2            /* port pin used for SCL */
#define LCD_SI           PD3            /* port pin used for SI (LCD's data input) */
#define LCD_CS           PD5            /* port pin used for /CS1 (optional) */
#define LCD_DOTS_X       128            /* number of horizontal dots */
#define LCD_DOTS_Y       64             /* number of vertical dots */
//#define LCD_FLIP_X                      /* enable horizontal flip */
#define LCD_OFFSET_X                    /* enable x offset of 4 dots */
#define LCD_FLIP_Y                      /* enable vertical flip */
#define LCD_START_Y      0              /* start line (0-63) */
#define LCD_CONTRAST     22             /* default contrast (0-63) */
#define FONT_8X8_V                      /* 8x8 font, vertically aligned */
#define SYMBOLS_24X24_VP                /* 24x24 symbols, vertically aligned */
//#endif



/*
 *  M12864 DIY Transistor Tester
 *  - ST7565 display
 *  - rotary encoder at PD1/3
 */

#if 0
#define LCD_ST7565R_SPI
#define LCD_GRAPHIC                     /* monochrome graphic display */
#define LCD_PORT         PORTD          /* port data register */
#define LCD_DDR          DDRD           /* port data direction register */
#define LCD_RESET        PD0            /* port pin used for /RES */
#define LCD_A0           PD1            /* port pin used for A0 */
#define LCD_SCL          PD2            /* port pin used for SCL */
#define LCD_SI           PD3            /* port pin used for SI (LCD's data input) */
#define LCD_DOTS_X       128            /* number of horizontal dots */
#define LCD_DOTS_Y       64             /* number of vertical dots */
//#define LCD_OFFSET_X                    /* enable x offset of 4 dots */
#define LCD_FLIP_Y                      /* enable vertical flip */
#define LCD_START_Y      0              /* start line (0-63) */
#define LCD_CONTRAST     11             /* default contrast (0-63) */
#define FONT_8X8_V                      /* 8x8 font, vertically aligned */
#define SYMBOLS_24X24_VP                /* 24x24 symbols, vertically aligned */
#endif



/*
 *  Chinese clone T3/T4 with ST7565 display
 *  - thanks to tom666 @ EEVblog forum 
 */

#if 0
#define LCD_ST7565R_SPI
#define LCD_GRAPHIC                     /* monochrome graphic display */
#define LCD_PORT         PORTD          /* port data register */
#define LCD_DDR          DDRD           /* port data direction register */
#define LCD_RESET        PD4            /* port pin used for /RES */
#define LCD_A0           PD3            /* port pin used for A0 */
#define LCD_SCL          PD2            /* port pin used for SCL */
#define LCD_SI           PD1            /* port pin used for SI (LCD's data input) */
#define LCD_CS           PD5            /* port pin used for /CS1 (optional) */
#define LCD_DOTS_X       128            /* number of horizontal dots */
#define LCD_DOTS_Y       64             /* number of vertical dots */
#define LCD_START_Y      0              /* start line (0-63) */
#define LCD_CONTRAST     11             /* default contrast (0-63) */
#define FONT_8X8_V                      /* 8x8 font, vertically aligned */
#define SYMBOLS_24X24_VP                /* 24x24 symbols, vertically aligned */
#endif



/*
 *  ILI9342, SPI interface
 */

#if 0
#define LCD_ILI9341_SPI
#define LCD_COLOR                       /* color graphic display */
#define LCD_PORT         PORTD          /* port data register */
#define LCD_DDR          DDRD           /* port data direction register */
#define LCD_RES          PD4            /* port pin used for /RES */
#define LCD_CS           PD5            /* port pin used for /CS */
#define LCD_DC           PD3            /* port pin used for D/C */
#define LCD_SCK          PD2            /* port pin used for SCK */
#define LCD_SDI          PD1            /* port pin used for SDI (LCD's data input) */
#define LCD_SDO          PD0            /* port pin used for SDO (LCD's data output) */
#define LCD_DOTS_X       320            /* number of horizontal dots */
#define LCD_DOTS_Y       240            /* number of vertical dots */
//#define LCD_FLIP_X                      /* enable horizontal flip */
//#define LCD_FLIP_Y                      /* enable vertical flip */
//#define LCD_ROTATE                      /* switch X and Y (rotate by 90�) */
#define FONT_16X26_H                    /* 16x26 font, horizontally aligned */
#define SYMBOLS_32X32_H                 /* 32x32 symbols, horizontally aligned */
#endif



/*
 *  ST7735, SPI interface
 */

#if 0
#define LCD_ST7735_SPI
#define LCD_COLOR                       /* color graphic display */
#define LCD_PORT         PORTD          /* port data register */
#define LCD_DDR          DDRD           /* port data direction register */
#define LCD_RES          PD4            /* port pin used for /RESX */
#define LCD_CS           PD5            /* port pin used for /CSX (optional) */
#define LCD_DC           PD3            /* port pin used for D/CX */
#define LCD_SCL          PD2            /* port pin used for SCL */
#define LCD_SDA          PD1            /* port pin used for SDA */
#define LCD_DOTS_X       128            /* number of horizontal dots */
#define LCD_DOTS_Y       160            /* number of vertical dots */
//#define LCD_FLIP_X                      /* enable horizontal flip */
#define LCD_FLIP_Y                      /* enable vertical flip */
#define LCD_ROTATE                      /* switch X and Y (rotate by 90�) */
#define FONT_10X16_H                    /* 10x16 font, horizontally aligned */
#define SYMBOLS_24X24_H                 /* 24x24 symbols, horizontally aligned */
#endif



/*
 *  PCD8544 (SPI interface)
 */

#if 0
#define LCD_PCD8544
#define LCD_GRAPHIC                     /* monochrome graphic display */
#define LCD_PORT         PORTD          /* port data register */
#define LCD_DDR          DDRD           /* port data direction register */
#define LCD_RES          PD4            /* port pin used for /RES */
#define LCD_SCE          PD5            /* port pin used for /SCE (optional) */
#define LCD_DC           PD3            /* port pin used for D/C */
#define LCD_SCLK         PD2            /* port pin used for SCLK */
#define LCD_SDIN         PD1            /* port pin used for SDIN (LCD's data input) */
#define LCD_DOTS_X       84             /* number of horizontal dots */
#define LCD_DOTS_Y       48             /* number of vertical dots */
#define LCD_CONTRAST     66             /* default contrast (1-127) */
#define FONT_6X8_V                      /* 6x8 font, vertically aligned */
#endif



/*
 *  check if a LCD module is specified
 */

#ifndef LCD_PORT
  #error <<< No LCD module specified! >>>
#endif



/* ************************************************************************
 *   touchscreen (optional)
 * ************************************************************************ */


/*
 *  touchscreen / controller
 *
 *  Please uncomment the package matching your touchscreen
 *  and adjust settings.
 *
 *  To uncomment, remove the enclosing "#if 0" and "#endif" or
 *  put a "//" in front of both.
 */


/*
 *  ADS7843 / XPT2046 (SPI interface)
 *  - not supported yet
 */

#if 0
#define TOUCH_ADS7843
#define TOUCH_PORT       PORTD     /* port data register */
#define TOUCH_DDR        DDRD      /* port data direction register */
#define TOUCH_CS                   /* port pin used for /CS */
#define TOUCH_D_CLK                /* port pin used for DCLK */
#define TOUCH_D_OUT                /* port pin used for DOUT */
#define TOUCH_D_IN                 /* port pin used for DIN */
#define TOUCH_PEN                  /* port pin used for /PENIRQ */
#endif




/* ************************************************************************
 *   port and pin assignments
 * ************************************************************************ */


/*
 *  Test probes:
 *  - Must be an ADC port :-)
 *  - Lower 3 pins of the port must be used for probe pins.
 *  - Please don't change the definitions of TP1, TP2 and TP3!
 */

#define ADC_PORT         PORTC     /* ADC port data register */
#define ADC_DDR          DDRC      /* ADC port data direction register */
#define ADC_PIN          PINC      /* port input pins register */
#define TP1              PC0       /* test pin 1 */
#define TP2              PC1       /* test pin 2 */
#define TP3              PC2       /* test pin 3 */

#define TP_ZENER         PC3       /* test pin with 10:1 voltage divider */
#define TP_REF           PC4       /* test pin with 2.5V reference and relay */
#define TP_BAT           PC5       /* test pin with 4:1 voltage divider */


/*
 *  Probe resistors
 *
 *  The resistors must be connected to the lower 6 pins of the port in
 *  following sequence:
 *  - pin 0: Rl1 680R (test pin 1)
 *  - pin 1: Rh1 470k (test pin 1)
 *  - pin 2: Rl2 680R (test pin 2)
 *  - pin 3: Rh2 470k (test pin 2)
 *  - pin 4: Rl3 680R (test pin 3)
 *  - pin 5: Rh3 470k (test pin 3)
 */

#define R_PORT           PORTB     /* port data register */
#define R_DDR            DDRB      /* port data direction register */


/*
 *  push button and power management
 */

#define CONTROL_PORT     PORTD     /* port data register */
#define CONTROL_DDR      DDRD      /* port data direction register */
#define CONTROL_PIN      PIND      /* port input pins register */
#define POWER_CTRL       PD6       /* controls power (1: on / 0: off) */
#define TEST_BUTTON      PD7       /* test/start push button (low active) */


/*
 *  rotary encoder
 */

#define ENCODER_PORT     PORTD     /* port data register */
#define ENCODER_DDR      DDRD      /* port data direction register */
#define ENCODER_PIN      PIND      /* port input pins register */
#define ENCODER_A        PD2       /* rotary encoder A signal */
#define ENCODER_B        PD3       /* rotary encoder B signal */



/* ************************************************************************
 *   internal stuff
 * ************************************************************************ */

/* ADC MUX channel for internal 1.1V bandgap reference */
#define ADC_BANDGAP      0x1e      /* 11110 */



/* ************************************************************************
 *   MCU specific setup to support different AVRs
 * ************************************************************************ */


/*
 *  ATmega664
 */

#if defined(__AVR_ATmega324__)

  /* estimated internal resistance of port to GND (in 0.1 Ohms) */
  #define R_MCU_LOW           200  /* 209 */

  /* estimated internal resistance of port to VCC (in 0.1 Ohms) */
  #define R_MCU_HIGH          220  /* 235 */

  /* voltage offset of MCUs analog comparator (in mV): -50 up to 50 */
  #define COMPARATOR_OFFSET   15

  /* capacitance of the probe tracks of the PCB and the MCU (in pF) */
  #define CAP_PCB             32

  /* this MCU has 32kB Flash, 1kB EEPROM and 2kB RAM (enable extra features) */
  #define RES_FLASH           32
  #define RES_EEPROM          1
  #define RES_RAM             2


/*
 *  ATmega664
 */

#elif defined(__AVR_ATmega644__)

  /* estimated internal resistance of port to GND (in 0.1 Ohms) */
  #define R_MCU_LOW           200  /* 209 */

  /* estimated internal resistance of port to VCC (in 0.1 Ohms) */
  #define R_MCU_HIGH          220  /* 235 */

  /* voltage offset of MCUs analog comparator (in mV): -50 up to 50 */
  #define COMPARATOR_OFFSET   15

  /* capacitance of the probe tracks of the PCB and the MCU (in pF) */
  #define CAP_PCB             32

  /* this MCU has 64kB Flash, 2kB EEPROM and 4kB RAM (enable extra features) */
  #define RES_FLASH           64
  #define RES_EEPROM          2
  #define RES_RAM             4


/*
 *  ATmega1284
 */

#elif defined(__AVR_ATmega1284__)

  /* estimated internal resistance of port to GND (in 0.1 Ohms) */
  #define R_MCU_LOW           200  /* 209 */

  /* estimated internal resistance of port to VCC (in 0.1 Ohms) */
  #define R_MCU_HIGH          220  /* 235 */

  /* voltage offset of MCUs analog comparator (in mV): -50 up to 50 */
  #define COMPARATOR_OFFSET   15

  /* capacitance of the probe tracks of the PCB and the MCU (in pF) */
  #define CAP_PCB             32

  /* this MCU has 128kB Flash, 4kB EEPROM and 16kB RAM (enable extra features) */
  #define RES_FLASH           128
  #define RES_EEPROM          4
  #define RES_RAM             16


/*
 *  missing or unsupported MCU
 */

#else
  #error <<< No or wrong MCU type selected! >>>
#endif



/* ************************************************************************
 *   EOF
 * ************************************************************************ */
