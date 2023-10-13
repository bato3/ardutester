/* ************************************************************************
 *
 *   driver functions for ILI9341 and ILI9342 compatible 
 *   color graphic displays
 *   - using SPI interface (4 line)
 *   - 240 x 320 (ILI9341) or 320 x 240 (ILI9342) pixels
 *
 *   (c) 2015-2016 by Markus Reschke
 *
 * ************************************************************************ */

/*
 *  hints:
 *  - pin assignment
 *    /RES     LCD_RES (optional)
 *    /CS      LCD_CS (optional)
 *    D/C      LCD_DC
 *    SCK      LCD_SCK
 *    SDI      LCD_SDI
 *    SDO      LCD_SDO (ILI9341 only, not used yet)
 *  - max. SPI clock: 10MHz, but up to 36 or 48MHz works also fine
 */


/* local includes */
#include "config.h"           /* global configuration */

#ifdef LCD_ILI9341_SPI


/*
 *  local constants
 */

/* source management */
#define LCD_DRIVER_C


/*
 *  include header files
 */

/* local includes */
#include "common.h"           /* common header file */
#include "variables.h"        /* global variables */
#include "functions.h"        /* external functions */
#include "colors.h"           /* color definitions */
#include "ILI9341.h"          /* ILI9341 specifics */

/* fonts and symbols, horizontally aligned */
#include "font_8x8_h.h"
#include "font_12x16_h.h"
#include "font_16x26_h.h"
#include "symbols_24x24_h.h"
#include "symbols_32x32_h.h"



/*
 *  derived constants
 */

/* maximum number of pixels for X and Y direction */
#ifdef LCD_ROTATE
  #define LCD_PIXELS_X        LCD_DOTS_Y
  #define LCD_PIXELS_Y        LCD_DOTS_X
#else
  #define LCD_PIXELS_X        LCD_DOTS_X
  #define LCD_PIXELS_Y        LCD_DOTS_Y
#endif

/* number of lines and characters per line */
#define LCD_CHAR_X            (LCD_PIXELS_X / FONT_SIZE_X)
#define LCD_CHAR_Y            (LCD_PIXELS_Y / FONT_SIZE_Y)

/* background color */
#define COLOR_BACKGROUND      COLOR_BLACK

/* component symbols */
#ifdef SW_SYMBOLS
  /* resize symbols by a factor of 2 */
  #define SYMBOL_RESIZE         2

  /* size in relation to a character */
  #define LCD_SYMBOL_CHAR_X   (((SYMBOL_SIZE_X * SYMBOL_RESIZE) + FONT_SIZE_X - 1) / FONT_SIZE_X)
  #define LCD_SYMBOL_CHAR_Y   (((SYMBOL_SIZE_Y * SYMBOL_RESIZE) + FONT_SIZE_Y - 1) / FONT_SIZE_Y)

  /* check y size: we need at least 2 lines */
  #if LCD_SYMBOL_CHAR_Y < 2
    #error <<< Symbols too small! >>>
  #endif
#endif



/*
 *  local variables
 */

/* address window */
uint16_t            X_Start;       /* start position X (column) */
uint16_t            X_End;         /* end position X (column) */
uint16_t            Y_Start;       /* start position Y (page/row) */
uint16_t            Y_End;         /* end position Y (page/row) */

/* text line management */
uint16_t            LineMask;      /* bit mask for up to 16 lines */

#ifdef SW_SYMBOLS
/* symbol positions (aligned to character positions) */
uint8_t             SymbolTop;     /* top line */
uint8_t             SymbolBottom;  /* bottom line */
uint8_t             SymbolLeft;    /* left of symbol */
uint8_t             SymbolRight;   /* right of symbol */
#endif



/* ************************************************************************
 *   low level functions for SPI interface
 * ************************************************************************ */



/*
 *  set up interface bus
 *  - should be called at firmware startup
 */

void LCD_BusSetup(void)
{
  uint8_t           Bits;          /* bitmask */


  /*
   *  set port pin's data direction
   */

  Bits = LCD_DDR;                       /* get current directions */

  /* basic output pins */
  Bits |= (1 << LCD_DC) | (1 << LCD_SCK) | (1 << LCD_SDI);

  /* optional output pins */
  #ifdef LCD_RES
    Bits |= (1 << LCD_RES);  
  #endif 
  #ifdef LCD_CS
    Bits |= (1 << LCD_CS);
  #endif

  Bits &= ~(1 << LCD_SDO);              /* basic input pins */

  LCD_DDR = Bits;                       /* set new directions */


  /*  set default levels:
   *  - /CS high, if pin available
   *  - /RES high, if pin available
   *  - SCK low
   */

  /* LCD_SCK should be low by default */

  /* optional pins */
  #ifdef LCD_CS
    /* disable chip */
    LCD_PORT |= (1 << LCD_CS);          /* set /CS high */
  #endif

  #ifdef LCD_RES
    /* disable reset */
    LCD_PORT = LCD_PORT | (1 << LCD_RES);    /* set /RES high */
  #endif
}



/*
 *  send a byte (data or command) to the LCD
 *  - bit-bang 8 bits, MSB first, LSB last
 *
 *  requires:
 *  - byte value to send
 */

void LCD_Send(uint8_t Byte)
{
  uint8_t           n = 8;         /* counter */

  /* select chip, if pin available */
  #ifdef LCD_CS
    LCD_PORT = LCD_PORT & ~(1 << LCD_CS);    /* set /CS1 low */
  #endif

  /* bit-bang 8 bits */
  while (n > 0)               /* for all bits */
  {
    /* start clock cycle (falling edge) and set data to 0 */
    LCD_PORT = LCD_PORT & ~((1 << LCD_SCK) | (1 << LCD_SDI));

    /* get current MSB and set SDI */
    if (Byte & 0b10000000)    /* 1 */
    {
      /* set SDI high */
      LCD_PORT = LCD_PORT | (1 << LCD_SDI);
    }

    /* end clock cycle (rising edge takes bit) */
    LCD_PORT = LCD_PORT | (1 << LCD_SCK);

    Byte <<= 1;               /* shift bits one step left */
    n--;                      /* next bit */
  }

  /* reset clock pin */
  LCD_PORT = LCD_PORT & ~(1 << LCD_SCK);     /* set clock low */

  /* deselect chip, if pin available */
  #ifdef LCD_CS
    LCD_PORT = LCD_PORT | (1 << LCD_CS);     /* set /CS1 high */
  #endif
}



/*
 *  send a command to the LCD
 *
 *  requires:
 *  - byte value to send
 */
 
void LCD_Cmd(uint8_t Cmd)
{
  /* indicate command mode */
  LCD_PORT = LCD_PORT & ~(1 << LCD_DC);      /* set D/C low */

  LCD_Send(Cmd);              /* send command */
}



/*
 *  send data to the LCD
 *
 *  requires:
 *  - byte value to send
 */

void LCD_Data(uint8_t Data)
{
  /* indicate data mode */
  LCD_PORT = LCD_PORT | (1 << LCD_DC);       /* set D/C high */

  LCD_Send(Data);             /* send data */
}



/*
 *  send data to the LCD
 *
 *  requires:
 *  - 2-byte value to send
 */

void LCD_Data2(uint16_t Data)
{
  uint8_t           n = 16;         /* counter */

  /* indicate data mode */
  LCD_PORT = LCD_PORT | (1 << LCD_DC);       /* set D/C high */

  /* select chip, if pin available */
  #ifdef LCD_CS
    LCD_PORT = LCD_PORT & ~(1 << LCD_CS);    /* set /CS1 low */
  #endif

  /* bit-bang 16 bits */
  while (n > 0)               /* for all bits */
  {
    /* start clock cycle (falling edge) and set data to 0 */
    LCD_PORT = LCD_PORT & ~((1 << LCD_SCK) | (1 << LCD_SDI));

    /* get current MSB and set SDI */
    if (Data & 0b1000000000000000)      /* 1 */
    {
      /* set SDI high */
      LCD_PORT = LCD_PORT | (1 << LCD_SDI);
    }

    /* end clock cycle (rising edge takes bit) */
    LCD_PORT = LCD_PORT | (1 << LCD_SCK);

    Data <<= 1;               /* shift bits one step left */
    n--;                      /* next bit */
  }

  /* reset clock pin */
  LCD_PORT = LCD_PORT & ~(1 << LCD_SCK);     /* set clock low */

  /* deselect chip, if pin available */
  #ifdef LCD_CS
    LCD_PORT = LCD_PORT | (1 << LCD_CS);     /* set /CS1 high */
  #endif
}



/* ************************************************************************
 *   high level functions
 * ************************************************************************ */



/*
 *  set address window
 *  - 0 up to (max - 1)
 */

void LCD_AddressWindow(void)
{
  /* X -> column */
  LCD_Cmd(CMD_COL_ADDR_SET);
  LCD_Data2(X_Start);               /* start column */
  LCD_Data2(X_End);                 /* end column */

  /* Y -> page */
  LCD_Cmd(CMD_PAGE_ADDR_SET);
  LCD_Data2(Y_Start);               /* start page */
  LCD_Data2(Y_End);                 /* end page */
}



/*
 *  set LCD character position
 *
 *  requires:
 *  - x:  horizontal position (1-)
 *  - y:  vertical position (1-)
 */

void LCD_CharPos(uint8_t x, uint8_t y)
{
  uint16_t          Mask = 1;

  /* update UI */
  UI.CharPos_X = x;
  UI.CharPos_Y = y;

  y--;                        /* start at zero */

  /* mark text line as used */
  if (y < 16)                 /* prevent overflow */
  {
    Mask <<= y;               /* shift to bit position for line */
    LineMask |= Mask;         /* set bit for line */
  }

  /* horizontal position (column) */
  x--;                        /* columns starts at 0 */
  Mask = x;                   /* expand to 16 bit */
  Mask *= FONT_SIZE_X;        /* offset for character */
  X_Start = Mask;             /* update start position */

  /* vertical position (page) */
  Mask = y;                   /* expand to 16 bit */
  Mask *= FONT_SIZE_Y;        /* offset for character */
  Y_Start = Mask;             /* update start position */
}



/*
 *  clear one single character line
 *
 *  requires:
 *  - Line: line number (1-)
 *    special case line 0: clear remaining space in current line
 */ 

void LCD_ClearLine(uint8_t Line)
{
  uint16_t          x = 0;         /* x position */
  uint8_t           y;             /* y position */
  uint8_t           Pos = 1;       /* character position */

  wdt_reset();                /* reset watchdog */

  if (Line == 0)         /* special case: rest of current line */
  {
    Line = UI.CharPos_Y;           /* get current line */
    Pos = UI.CharPos_X;            /* get current character position */
  }

  /* have we to clear this line? */
  if (Line <= 16)                  /* prevent overflow */
  {
    y = Line - 1;                  /* bitmask starts at zero */
    x = 1;                         /* set start bit */
    x <<= y;                       /* bit for this line */

    if (! (LineMask & x))          /* bit not set */
    {
      return;                      /* nothing do to */
    }
  }

  /* manage address window */
  LCD_CharPos(Pos, Line);         /* update character position */
                                  /* also updates X_Start and Y_Start */
  if (Pos == 1)                   /* complete line */
  {
    if (x > 0)                    /* got line bit */
    {
      LineMask &= ~x;             /* clear bit */
    }
  }

  X_End = LCD_PIXELS_X - 1;             /* last column */
  Y_End = Y_Start + FONT_SIZE_Y - 1;    /* last row */
  y = FONT_SIZE_Y;                      /* set default */

  /* partial text line at bottom of display */
  if (Y_End > (LCD_PIXELS_Y - 1))       /* row overflow */
  {
    x = Y_End - (LCD_PIXELS_Y - 1);     /* difference */
    y -= (uint8_t)x;                    /* adjust number of rows */
    Y_End = LCD_PIXELS_Y - 1;           /* set last row */    
  }

  LCD_AddressWindow();                  /* set window */

  /* send background color */
  LCD_Cmd(CMD_MEM_WRITE);          /* start writing */

  while (y > 0)                    /* character height (pages) */
  {
    x = X_Start;                   /* reset start position */
    while (x < LCD_PIXELS_X)       /* all columns */
    {
      /* send background color */
      LCD_Data2(COLOR_BACKGROUND);

      x++;                         /* next column */
    }

    y--;                           /* next page */
  }
}



/*
 *  clear the display 
 */ 

void LCD_Clear(void)
{
  uint8_t           n = 1;         /* counter */

  /* we have to clear all dots manually :-( */
  while (n <= (LCD_CHAR_Y + 1))    /* for all text lines */
  {
    /* +1 is for a possible partial line at the bottom */

    LCD_ClearLine(n);              /* clear line */
    n++;                           /* next line */
  }

  LCD_CharPos(1, 1);          /* reset character position */
}



/*
 *  initialize LCD
 */
 
void LCD_Init(void)
{
  uint8_t           Bits;

  /* hardware reset */
  #ifdef LCD_RES
  LCD_PORT = LCD_PORT & ~(1 << LCD_RES);     /* set /RES low */
  wait10us();                                /* wait 10�s */
  LCD_PORT = LCD_PORT | (1 << LCD_RES);      /* set /RES high */
  /* blanking sequence needs up to 120ms */
  /* but we may send command after 5ms */
  MilliSleep(5);                             /* wait 5ms */
  #endif


  /* 
   *  set registers
   */

  /* power control A */
  LCD_Cmd(CMD_POWER_CTRL_A);
  LCD_Data(MASK_POWER_CTRL_A_1);        /* fixed value */
  LCD_Data(MASK_POWER_CTRL_A_2);        /* fixed value */
  LCD_Data(MASK_POWER_CTRL_A_3);        /* fixed value */
  LCD_Data(MASK_POWER_CTRL_A_4 | FLAG_REG_VD_160);     /* Vcore 1.6V */
  LCD_Data(FLAG_VBC_56);                               /* DDVDH 5.6V */

  /* power control B */  
  LCD_Cmd(CMD_POWER_CTRL_B);
  LCD_Data(MASK_POWER_CTRL_B_1);        /* fixed value */
  LCD_Data(MASK_POWER_CTRL_B_2 | FLAG_POWER_CTRL_B);
  LCD_Data(MASK_POWER_CTRL_B_3 | FLAG_DC_ON);          /* ESD protection on */

  /* pump ratio control */
  LCD_Cmd(CMD_PUMP_RATIO_CTRL);
  LCD_Data(FLAG_PUMP_RATIO_2);     /* 2xVCI */

  /* power control 1 */
  LCD_Cmd(CMD_POWER_CTRL_1);
  LCD_Data(FLAG_VRH_460);          /* GVDD 4.60V */

  /* power control 2 */
  LCD_Cmd(CMD_POWER_CTRL_2);
  LCD_Data(FLAG_BT_3);             /* DDVDH=2*VCI, VGH=6xVCI, VGL=-3*VCI */

  /* VCOM control 1 */
  LCD_Cmd(CMD_VCOM_CTRL_1);
  LCD_Data(FLAG_VMH_5000);         /* 5.000V, could be used to adjust contrast */
  LCD_Data(FLAG_VML_0600);         /* -0.600V */

  /* VCOM control 2 */
  LCD_Cmd(CMD_VCOM_CTRL_2);
  LCD_Data(FLAG_VMF_M44 | FLAG_NVM_1);       /* -44 */

  /* driver timing control A */
  LCD_Cmd(CMD_TIME_CTRL_A);
  LCD_Data(MASK_TIME_CTRL_A_1 | FLAG_NOW_1);      /* default + 1 unit */
  LCD_Data(FLAG_CR_1);                            /* default CR timing */
  LCD_Data(MASK_TIME_CTRL_A_3 | FLAG_PC_0);       /* default - 2 units */

  /* driver timing control B */
  LCD_Cmd(CMD_TIME_CTRL_B);
  LCD_Data(FLAG_VG_SW_T1_0 | FLAG_VG_SW_T2_0 | FLAG_VG_SW_T3_0 | FLAG_VG_SW_T4_0);
  LCD_Data(MASK_TIME_CTRL_B_2);                   /* fixed value */

  /* set pixel format for RGB image data */
  LCD_Cmd(CMD_PIX_FORMAT_SET);
  LCD_Data(FLAG_DBI_16);           /* 16 Bits per pixel */

  /* frame control for normal display mode */
  LCD_Cmd(CMD_FRAME_CTRL_NORM);
  LCD_Data(FLAG_DIVA_1);           /* f_OSC */
  LCD_Data(FLAG_RTNA_24);          /* 24 clocks */

  /* display function control */
  LCD_Cmd(CMD_FUNC_CTRL);
  LCD_Data(FLAG_PT_0);                  /* V63 / V0 - VCOML / VCOMH */
  LCD_Data(FLAG_REV_1 | FLAG_ISC_01);   /* white, 1 frame */
  LCD_Data(FLAG_NL_320);                /* 320 lines */
  LCD_Data(0x00);                       /* DOTCLK / 2 */

  /* gamma set */
  LCD_Cmd(CMD_GAMMA_SET);
  LCD_Data(FLAG_GC_1);             /* gamma curve 1 */

  /* memory access control */
  LCD_Cmd(CMD_MEM_CTRL);
  Bits = FLAG_BGR_RGB;             /* color bit order: RGB */
  #ifdef LCD_ROTATE
    Bits |= FLAG_MV_REV;           /* swap x and y */
  #endif
  #ifdef LCD_FLIP_X 
    Bits |= FLAG_MX_REV;           /* flip x */
  #endif
  #ifdef LCD_FLIP_Y
    Bits |= FLAG_MY_REV;           /* flip y */
  #endif
  LCD_Data(Bits);

  /* address window */
  X_Start = 0;
  X_End = LCD_PIXELS_X - 1;
  Y_Start = 0;
  Y_End = LCD_PIXELS_Y - 1;
  LCD_AddressWindow();

  /* power on */
  MilliSleep(120);                 /* pause for 120ms */
  LCD_Cmd(CMD_SLEEP_OUT);          /* leave sleep mode */
  MilliSleep(60);                  /* pause for 60ms */
  LCD_Cmd(CMD_DISPLAY_ON);         /* enable display output */
  MilliSleep(80);                  /* pause for 80ms */

  /* update maximums */
  UI.CharMax_X = LCD_CHAR_X;       /* characters per line */
  UI.CharMax_Y = LCD_CHAR_Y;       /* lines */

  /* set default color in case the color feature is disabled */
  #ifndef LCD_COLOR
    UI.PenColor = COLOR_GREEN;     /* set pen color */
  #endif

  LineMask = 0xffff;            /* clear all lines by default */
  LCD_CharPos(1, 1);            /* reset character position */
  /* we don't clear the display now, because it's quite slow */ 
}



/*
 *  display a single character
 *
 *  requires:
 *  - Char: character to display
 */

void LCD_Char(unsigned char Char)
{
  uint8_t           *Table;        /* pointer to table */
  uint8_t           Index;         /* font index */
  uint16_t          Offset;        /* address offset */
  uint8_t           Pixels;        /* pixels in y direction */
  uint8_t           x;             /* bitmap x byte counter */
  uint8_t           y = 1;         /* bitmap y byte counter */
  uint8_t           Bits;          /* number of bits to be sent */
  uint8_t           n;             /* bitmap bit counter */

  /* get font index number from lookup table */
  Table = (uint8_t *)&FontTable;        /* start address */
  Table += Char;                        /* add offset for character */
  Index = pgm_read_byte(Table);         /* get index number */
  if (Index == 0xff) return;            /* no character bitmap available */

  /* calculate start address of character bitmap */
  Table = (uint8_t *)&FontData;        /* start address of font data */
  Offset = FONT_BYTES_N * Index;       /* offset for character */
  Table += Offset;                     /* address of character data */

  /* LCD's address window */
  LCD_CharPos(UI.CharPos_X, UI.CharPos_Y);   /* update character position */
                                             /* also updates X_Start and Y_Start */
  X_End = X_Start + FONT_SIZE_X - 1;   /* offset for end */
  Y_End = Y_Start + FONT_SIZE_Y - 1;   /* offset for end */
  LCD_AddressWindow();                 /* set address window */

  Offset = UI.PenColor;                /* get pen color */
  LCD_Cmd(CMD_MEM_WRITE);              /* start writing */

  /* read character bitmap and send it to display */
  while (y <= FONT_BYTES_Y)
  {
    Pixels = FONT_SIZE_X;               /* track x bits to be sent */
    x = 1;                              /* reset counter */

    /* read and send all bytes for this row */
    while (x <= FONT_BYTES_X)
    {
      /* track x bits */
      if (Pixels >= 8) Bits = 8;
      else Bits = Pixels;
      Pixels -= Bits;

      Index = pgm_read_byte(Table);     /* read byte */

      /* send color for each bit */
      n = Bits;
      while (n > 0)
      {
        if (Index & 0b00000001)         /* bit set */
        {
          LCD_Data2(Offset);            /* foreground color */
        }
        else                            /* bit unset */
        {
          LCD_Data2(COLOR_BACKGROUND);  /* background color */
        }

        Index >>= 1;                      /* shift byte for next bit */
        n--;                              /* next bit */
      }

      Table++;                          /* address for next byte */
      x++;                              /* next byte */
    }

    y++;                                /* next row */
  }

  UI.CharPos_X++;             /* update character position */
}



/*
 *  set cursor
 *
 *  required:
 *  - Mode: cursor mode
 *    0: cursor on
 *    1: cursor off
 */

void LCD_Cursor(uint8_t Mode)
{
  LCD_CharPos(LCD_CHAR_X, LCD_CHAR_Y);     /* move to bottom right */

  if (Mode)              /* cursor on */
  {
    LCD_Char('>');
  }
  else                   /* cursor off */
  {
    LCD_Char(' ');
  }
}



/* ************************************************************************
 *   special stuff
 * ************************************************************************ */


#ifdef SW_SYMBOLS

/*
 *  display a component symbol
 *
 *  requires:
 *  - ID: symbol to display
 */

void LCD_Symbol(uint8_t ID)
{
  uint8_t           *Table;        /* pointer to symbol table */
  uint8_t           *Table2;       /* pointer */
  uint8_t           Data;          /* symbol data */
  uint16_t          Offset;        /* address offset */
  uint8_t           Pixels;        /* pixels in y direction */
  uint8_t           x;             /* bitmap x byte counter */
  uint8_t           y = 1;         /* bitmap y byte counter */
  uint8_t           Bits;          /* number of bits to be sent */
  uint8_t           n;             /* bitmap bit counter */
  uint8_t           factor = SYMBOL_RESIZE;  /* resize factor */

  /* calculate start address of character bitmap */
  Table = (uint8_t *)&SymbolData;       /* start address of symbol data */
  Offset = SYMBOL_BYTES_N * ID;         /* offset for symbol */
  Table += Offset;                      /* address of symbol data */

  /* LCD's address window */
  LCD_CharPos(UI.CharPos_X, UI.CharPos_Y);   /* update character position */
                                             /* also updates X_Start and Y_Start */
  X_End = X_Start + (SYMBOL_SIZE_X * SYMBOL_RESIZE) - 1;  /* offset for end */
  Y_End = Y_Start + (SYMBOL_SIZE_Y * SYMBOL_RESIZE) - 1;  /* offset for end */
  LCD_AddressWindow();                  /* set address window */

  Offset = UI.PenColor;                 /* get pen color */
  LCD_Cmd(CMD_MEM_WRITE);               /* start writing */

  /* read character bitmap and send it to display */
  while (y <= SYMBOL_BYTES_Y)
  {
    Table2 = Table;           /* save current pointer */

    while (factor > 0)        /* resize symbol */
    {
      Table = Table2;                   /* reset start pointer */

      Pixels = SYMBOL_SIZE_X;           /* track x bits to be sent */
      x = 1;                            /* reset counter */

      /* read and send all bytes for this row */
      while (x <= SYMBOL_BYTES_X)
      {
        /* track x bits */
        if (Pixels >= 8) Bits = 8;
        else Bits = Pixels;
        Pixels -= Bits;

        Data = pgm_read_byte(Table);    /* read byte */

        /* send color for each bit */
        n = Bits;                       /* reset counter */
        n *= SYMBOL_RESIZE;             /* and consider size factor */

        while (n > 0)                   /* x pixels */
        {
          if (Data & 0b00000001)             /* bit set */
          {
            LCD_Data2(Offset);               /* foreground color */
          }
          else                               /* bit unset */
          {
            LCD_Data2(COLOR_BACKGROUND);     /* background color */
          }

          n--;                          /* next pixel */

          if (n % SYMBOL_RESIZE == 0)   /* for every resize step */
          {
            Data >>= 1;                 /* shift byte for next bit */
          }
        }

        Table++;                        /* address for next byte */
        x++;                            /* next byte */
      }

      factor--;                    /* one part done */
    }

    if (factor == 0)               /* all parts done */
    {
      factor = SYMBOL_RESIZE;      /* reset resize factor */
      y++;                         /* next row */
    }              
  }

  /* mark text lines as used */
  n = LCD_SYMBOL_CHAR_Y;           /* set line counter */
  x = SymbolTop;                   /* start line */
  while (n > 1)                    /* first line already set */
  {
    x++;                           /* next line */
    LCD_CharPos(1, x);             /* mark line */
    n--;                           /* next line */
  }
}



/*
 *  display fancy probe number
 *
 *  requires:
 *  - Probe: probe number
 *  - Table: pointer to pinout details
 */

void LCD_FancyProbeNumber(uint8_t Probe, uint8_t *Table)
{
  uint8_t           Data;          /* pinout data */
  uint8_t           x;             /* x position */
  uint8_t           y;             /* y position */

  Data = pgm_read_byte(Table);     /* read pinout details */

  if (Data != PIN_NONE)            /* show pin */
  {
    /* determine position based on pinout data */
    x = SymbolLeft;         /* set default positions */
    y = SymbolTop;
    if (Data & PIN_RIGHT) x = SymbolRight;
    if (Data & PIN_BOTTOM) y = SymbolBottom;

    /* show probe number */
    LCD_CharPos(x, y);           /* set position */
    LCD_ProbeNumber(Probe);      /* display probe number */
  }
}



/*
 *  show fancy pinout for semiconductors
 *  - display a nice symbol in the middle of the bottom text lines
 *  - display pin numbers left and right of symbol
 *  - symbol ID (0-) in Check.Symbol
 */

void LCD_FancySemiPinout(void)
{
  uint8_t           n;             /* temp. value */
  uint8_t           Line;          /* line number */
  uint8_t           *Table;        /* pointer to pin table */
  uint16_t          Offset;        /* address offset */

  /* check if we got enough unused lines left on the display */
  Line = UI.CharPos_Y;        /* current text line */
  n = LCD_CHAR_Y;             /* number of text lines */
  n = n - Line;               /* free lines left */
  if (n < LCD_SYMBOL_CHAR_Y) return;    /* too few lines */
  if (n > LCD_SYMBOL_CHAR_Y) Line++;    /* add a spacer line */

  /* determine positions */
  Line++;                               /* next line */
  SymbolTop = Line;
  SymbolBottom = Line;
  SymbolBottom += (LCD_SYMBOL_CHAR_Y - 1);   /* add offset for symbol */
  SymbolLeft = (LCD_CHAR_X - LCD_SYMBOL_CHAR_X) / 2;
  SymbolRight = SymbolLeft;
  SymbolRight += LCD_SYMBOL_CHAR_X + 1;      /* add offset for symbol */

  /* calculate start address of pinout details */
  Table = (uint8_t *)&PinTable;         /* start address of pin table */
  Offset = Check.Symbol * 3;            /* offset for pin details */
  Table += Offset;                      /* address of pin details */

  /* display pin numbers */
  LCD_FancyProbeNumber(Semi.A, Table);       /* A pin */
  Table++;                                   /* details for next pin */
  LCD_FancyProbeNumber(Semi.B, Table);       /* B pin */
  Table++;                                   /* details for next pin */
  LCD_FancyProbeNumber(Semi.C, Table);       /* C pin */

  /* display symbol */
  #ifdef LCD_COLOR
    Offset = UI.PenColor;               /* save color */
    UI.PenColor = COLOR_YELLOW;         /* set pen color */
  #endif

  LCD_CharPos(SymbolLeft + 1, SymbolTop);    /* set top left position  */
  LCD_Symbol(Check.Symbol);             /* display symbol */

  #ifdef LCD_COLOR
    UI.PenColor = Offset;          /* restore pen color */
  #endif
}

#endif



/* ************************************************************************
 *   clean-up of local constants
 * ************************************************************************ */

/* source management */
#undef LCD_DRIVER_C

#endif

/* ************************************************************************
 *   EOF
 * ************************************************************************ */
