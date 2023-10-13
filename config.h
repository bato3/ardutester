/* ************************************************************************
 *
 *   global configuration, setup and settings
 *
 *   (c) 2012-2015 by Markus Reschke
 *   based on code from Markus Frejek and Karl-Heinz Kübbeler
 *
 * ************************************************************************ */


/* source management */
#define CONFIG_H



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
 */

#if 0
#define LCD_HD44780_PAR4
#define LCD_TEXT                        /* character display */
#define LCD_PORT         PORTD          /* port data register */
#define LCD_DDR          DDRD           /* port data direction register */
                                        /* - lower 4 bits for LCD data interface */
                                        /* - upper 4 bits for control lines */
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
#define SYMBOLS_24X24_V                 /* 24x24 symbols, vertically aligned */
//#endif



/*
 *  M12864 DIY Transistor Tester
 *  - ST7585 display
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
#define LCD_OFFSET_X                    /* enable x offset of 4 dots */
#define LCD_FLIP_Y                      /* enable vertical flip */
#define LCD_START_Y      0              /* start line (0-63) */
#define LCD_CONTRAST     11             /* default contrast (0-63) */
#define FONT_8X8_V                      /* 8x8 font, vertically aligned */
#define SYMBOLS_24X24_V                 /* 24x24 symbols, vertically aligned */
#endif



/*
 *  Chinese clone T3/T4 with ST7585 display
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
#define SYMBOLS_24X24_V                 /* 24x24 symbols, vertically aligned */
#endif



/*
 *  ILI8342, SPI interface
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
//#define LCD_ROTATE                      /* switch X and Y (rotate by 90°) */
#define LCD_SLOW                        /* slow interaction */
#define FONT_16X26_H                    /* 16x26 font, horizontally aligned */
#define SYMBOLS_32X32_H                 /* 32x32 symbols, horizontally aligned */
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
 *   Hardware options
 * ************************************************************************ */


/*
 *  rotary encoder for user interface (PD2 & PD3)
 *  - in parallel with LCD module
 *  - requires MCU with >=32kB Flash
 *  - uncomment to enable and also set ENCODER_PULSES below to match your
 *    rotary encoder
 */

//#define HW_ENCODER


/*
 *  Number of pulses per step or detent for the rotary encoder
 *  - typical values: 1, 2 or 4
 *  - adjust value to match your rotary encoder
 */

#define ENCODER_PULSES   2


/*
 *  2.5V voltage reference for Vcc check (PC4)
 *  - should be at least 10 times more precise than the voltage regulator
 *  - uncomment to enable and also adjust UREF_25 below for your voltage
 *    reference
 */

//#define HW_REF25


/*
 *  Typical voltage of 2.5V voltage reference (in mV)
 *  - see datasheet of the voltage reference
 *  - or use >= 5.5 digit DMM to measure the voltage
 */

#define UREF_25           2495


/*
 *  Probe protection relay for discharging caps (PC4):
 *  - low signal: short circuit probe pins
 *    high signal via external reference: remove short circuit 
 *  - uncomment to enable
 */

//#define HW_RELAY


/*
 *  voltage measurement up to 50V DC (10:1 voltage divider, PC3):
 *  - for Zener diodes
 *  - DC-DC boost converter controled by test push button
 *  - requires MCU with >=32kB Flash and >=1kB EEPROM
 *  - uncomment to enable
 */

//#define HW_ZENER


/*
 *  frequency counter (PD4)
 *  - in parallel with LCD module
 *  - requires MCU with >=32kB Flash
 *  - uncomment to enable
 */

//#define HW_FREQ_COUNTER



/* ************************************************************************
 *   software options
 * ************************************************************************ */


/*
 *  PWM generator
 *  - uncomment to enable
 */

#define SW_PWM


/*
 *  Inductance measurement
 *  - uncomment to enable
 */

#define SW_INDUCTOR


/*
 *  ESR measurement and in-circuit ESR measurement
 *  - uncomment to enable
 */

#define SW_ESR


/*
 *  Check for rotary encoders
 *  - uncomment to enable
 */

#define SW_ENCODER


/*
 *  squarewave signal generator
 *  - requires rotary encoder
 *  - uncomment to enable
 */

#define SW_SQUAREWAVE


/*
 *  IR remote control detection
 *  - requires TSOP IR receiver module
 *  - uncomment to enable
 */

#define SW_IR_RECEIVER



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
#define TP1              PC0       /* test pin 1 (=0) */
#define TP2              PC1       /* test pin 2 (=1) */
#define TP3              PC2       /* test pin 3 (=2) */

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
#define ENCODER_A        PD2       /* rotary encoder A signal */
#define ENCODER_B        PD3       /* rotary encoder B signal */


/* ************************************************************************
 *   Makefile workaround for some IDEs 
 * ************************************************************************ */


/*
 *  Oscillator startup cycles (after wakeup from power-safe mode):
 *  - typical values
 *    - internal RC:              6
 *    - full swing crystal:   16384 (also 256 or 1024 based on fuse settings)
 *    - low power crystal:    16384 (also 256 or 1024 based on fuse settings)
 *  - Please change value if it doesn't match your tester!
 */

#ifndef OSC_STARTUP
  #define OSC_STARTUP    16384
#endif



/* ************************************************************************
 *   misc settings
 * ************************************************************************ */


/*
 *  Languange of user interface. Available languages:
 *  - English (default)
 *  - German
 *  - Czech
 */

#define UI_ENGLISH
//#define UI_GERMAN
//#define UI_CZECH


/*
 *  LCD module with cyrillic character set
 *  - uncomment if you are using such an LCD
 */

//#define LCD_CYRILLIC


/*
 *  Maximum time to wait after a measurement in continous mode (in ms).
 *  - Time between printing the result and starting a new cycle.
 */

#define CYCLE_DELAY      3000


/*
 *  Maximum number of measurements without any components found.
 *  - If that number is reached the tester powers off.
 */

#define CYCLE_MAX        5


/*
 *  Voltage drop by reverse voltage protection diode and power management.
 *  transistor (in mV):
 *  - Schottky diode about 200mV / Transistor about 100mV.
 *  - Get your DMM and measure the voltage drop!
 *  - Could be also used to compensate any offset by the voltage divider
 *    used to measure the battery voltage.
 */  

#define BAT_OFFSET       290


/*
 *  Battery low voltage (in mV).
 *  - Tester warns if BAT_POOR + 1V is reached.
 *  - Tester powers off if BAT_POOR is reached.
 *  - Voltage drop (BAT_OUT) is considered in calculation.
 */

#define BAT_POOR         6400



/* ************************************************************************
 *   measurement settings and offsets
 * ************************************************************************ */


/*
 *  ADC voltage reference based on Vcc (in mV). 
 */

#define UREF_VCC         5001


/*
 * Offset for the internal bandgap voltage reference (in mV): -100 up to 100
 *  - To compensate any difference between real value and measured value.
 *  - The ADC has a resolution of about 4.88mV for V_ref = 5V (Vcc) and
 *    1.07mV for V_ref = 1.1V (bandgap).
 *  - Will be added to measured voltage of bandgap reference.
 */

#define UREF_OFFSET      0


/*
 *  Exact values of probe resistors.
 *  - Standard value for Rl is 680 Ohms.
 *  - Standard value for Rh is 470k Ohms.
 */

/* Rl in Ohms */
#define R_LOW            680

/* Rh in Ohms */
#define R_HIGH           470000


/*
 *  Offset for systematic error of resistor measurement with Rh (470k) 
 *  in Ohms.
 */

#define RH_OFFSET        700 



/*
 *  Resistance of probe leads (in 0.01 Ohms).
 *  - Resistance of two probe leads in series.
 *  - Assuming all probe leads got same/similar resistance.
 */

#define R_ZERO           20


/* 
 *  Capacitance of the wires between PCB and terminals (in pF).
 *  Examples:
 *  - 2pF for wires 10cm long
 */

#define CAP_WIRES        2


/* 
 *  Capacitance of the probe leads connected to the tester (in pF).
 *  Examples:
 *    capacity  length of probe leads
 *    -------------------------------
 *     3pF      about 10cm
 *     9pF      about 30cm
 *    15pF      about 50cm
 */

#define CAP_PROBELEADS   9


/*
 *  Maximum voltage at which we consider a capacitor being
 *  discharged (in mV)
 */

#define CAP_DISCHARGED   2


/*
 *  Number of ADC samples to perform for each mesurement.
 *  - Valid values are in the range of 1 - 255.
 */

#define ADC_SAMPLES      25



/* ************************************************************************
 *   MCU specific setup to support different AVRs
 * ************************************************************************ */


/*
 *  ATmega328
 */

#if defined(__AVR_ATmega328__)

  /* estimated internal resistance of port to GND (in 0.1 Ohms) */
  #define R_MCU_LOW           200  /* 209 */

  /* estimated internal resistance of port to VCC (in 0.1 Ohms) */
  #define R_MCU_HIGH          220  /* 235 */

  /* voltage offset of MCUs analog comparator (in mV): -50 up to 50 */
  #define COMPARATOR_OFFSET   15

  /* capacitance of the probe tracks of the PCB and the MCU (in pF) */
  #define CAP_PCB             32

  /* total default capacitance (in pF): max. 255 */
  #define C_ZERO              CAP_PCB + CAP_WIRES + CAP_PROBELEADS

  /* this MCU has 32kB Flash, 1kB EEPROM and 2kB RAM (enable extra features) */
  #define RES_FLASH           32
  #define RES_EEPROM          1
  #define RES_RAM             2


/*
 *  ATmega664   64k flash, 2k EEPROM, 4k RAM
 *  ATmega1284  128k flash, 4k EEPROM, 16k RAM
 */


/*
 *  missing or unsupported MCU
 */

#else
  #error <<< No or wrong MCU type selected! >>>
#endif



/* ************************************************************************
 *   ADC clock
 * ************************************************************************ */


/*
 *  selection of ADC clock 
 *  - ADC clock can be 125000 or 250000 
 *  - 250kHz is out of the full accuracy specification!
 */

#define ADC_FREQ    125000


/*
 *  define clock divider
 *  - supports 1MHz, 2MHz, 4MHz, 8MHz and 16MHz MCU clock
 *  - 4 for CPU clock of 1MHz and ADC clock of 250kHz
 *  - 128 for CPU clock of 16MHz and ADC clock of 125kHz
 */

#define CPU_FREQ    F_CPU

#if CPU_FREQ / ADC_FREQ == 4
  #define ADC_CLOCK_DIV (1 << ADPS1) 
#endif

#if CPU_FREQ / ADC_FREQ == 8
  #define ADC_CLOCK_DIV (1 << ADPS1) | (1 << ADPS0)
#endif

#if CPU_FREQ / ADC_FREQ == 16
  #define ADC_CLOCK_DIV (1 << ADPS2)
#endif

#if CPU_FREQ / ADC_FREQ == 32
  #define ADC_CLOCK_DIV (1 << ADPS2) | (1 << ADPS0)
#endif

#if CPU_FREQ / ADC_FREQ == 64
  #define ADC_CLOCK_DIV (1 << ADPS2) | (1 << ADPS1)
#endif

#if CPU_FREQ / ADC_FREQ == 128
  #define ADC_CLOCK_DIV (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)
#endif


/* ************************************************************************
 *   ressource management
 * ************************************************************************ */


/*
 *  software options
 */

/* squarewave generator requires rotary encoder */
#ifdef SW_SQUAREWAVE
  #ifndef HW_ENCODER
    #undef SW_SQUAREWAVE
  #endif
#endif

/* LCD module */
#ifdef LCD_CONTRAST
  #define SW_CONTRAST
#else
  #define LCD_CONTRAST        0
#endif

/* component symbols for fancy pinout */
#if defined (SYMBOLS_24X24_V) || defined (SYMBOLS_24X24_H) || defined (SYMBOLS_32X32_H)
  #define SW_SYMBOLS
#endif

/* touchscreen */
#ifdef TOUCH_PORT
  #define HW_TOUCH
#endif



/* ************************************************************************
 *   EOF
 * ************************************************************************ */
