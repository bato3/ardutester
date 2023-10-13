/* ************************************************************************
 *
 *   old 32x32 component symbols 
 *   horizontally aligned, horizontal bit order flipped
 *
 *   (c) 2015-2022 by Markus Reschke
 *
 * ************************************************************************ */


/* ************************************************************************
 *   symbol data
 * ************************************************************************ */

#ifdef SYMBOLS_32X32_OLD_HF


/* source management */
#define SYMBOL_SET                 /* symbol set included */

/* symbol size */
#define SYMBOL_SIZE_X       32     /* width:  32 dots */
#define SYMBOL_SIZE_Y       32     /* height: 32 dots */

/* symbol data format */
#define SYMBOL_BYTES_N     128     /* 128 bytes per symbol */
#define SYMBOL_BYTES_X       4     /* 4 bytes in x direction */
#define SYMBOL_BYTES_Y      32     /* 32 bytes in y direction */


/*
 *  symbol bitmaps
 *  - format:
 *    - 128 bytes per symbol
 *    - first four bytes: first horizontal line (top to down)
 *      first byte: left part
 *    - bit #0: left / bit #7: right (horizontally flipped)
 */

const uint8_t SymbolData[] PROGMEM = {
  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x00,0x20,0x02,0x40,0x18,0x10,0x04,
  0x20,0x18,0x08,0x08,0x20,0x18,0x04,0x08,0x10,0x18,0x02,0x10,0x10,0x18,0x01,0x10,
  0x10,0x98,0x00,0x10,0x08,0x58,0x00,0x20,0x08,0x38,0x00,0x20,0xFF,0x1F,0x00,0x20,
  0x08,0x38,0x00,0x20,0x08,0x58,0x00,0x20,0x10,0x98,0x00,0x10,0x10,0x18,0x09,0x10,
  0x10,0x18,0x0E,0x10,0x20,0x18,0x0E,0x08,0x20,0x18,0x0F,0x08,0x40,0x18,0x10,0x04,
  0x80,0x00,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* BJT npn */

  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x00,0x20,0x02,0x40,0x00,0x10,0x04,
  0x20,0x18,0x09,0x08,0x20,0x18,0x07,0x08,0x10,0x18,0x07,0x10,0x10,0x18,0x0F,0x10,
  0x10,0x98,0x00,0x10,0x08,0x58,0x00,0x20,0x08,0x38,0x00,0x20,0xFF,0x1F,0x00,0x20,
  0x08,0x38,0x00,0x20,0x08,0x58,0x00,0x20,0x10,0x98,0x00,0x10,0x10,0x18,0x01,0x10,
  0x10,0x18,0x02,0x10,0x20,0x18,0x04,0x08,0x20,0x18,0x08,0x08,0x40,0x00,0x10,0x04,
  0x80,0x00,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* BJT pnp */

  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x20,0x20,0x02,0x40,0x20,0x20,0x04,
  0x20,0xE8,0x3F,0x08,0x20,0x28,0x00,0x08,0x10,0x28,0x00,0x10,0x10,0x08,0x00,0x10,
  0x10,0x08,0x00,0x10,0x08,0x28,0x04,0x20,0x08,0x28,0x06,0x20,0x08,0xE8,0x3F,0x20,
  0x08,0x28,0x26,0x20,0x08,0x28,0x24,0x20,0x10,0x08,0x20,0x10,0x10,0x08,0x20,0x10,
  0x10,0x28,0x20,0x10,0x20,0x28,0x20,0x08,0xFF,0xEF,0x3F,0x08,0x40,0x20,0x20,0x04,
  0x80,0x20,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* MOSFET enh n-ch */

  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x20,0x20,0x02,0x40,0x20,0x20,0x04,
  0xFF,0xEF,0x3F,0x08,0x20,0x28,0x20,0x08,0x10,0x28,0x20,0x10,0x10,0x08,0x20,0x10,
  0x10,0x08,0x20,0x10,0x08,0x28,0x22,0x20,0x08,0x28,0x26,0x20,0x08,0xE8,0x3F,0x20,
  0x08,0x28,0x06,0x20,0x08,0x28,0x02,0x20,0x10,0x08,0x00,0x10,0x10,0x08,0x00,0x10,
  0x10,0x28,0x00,0x10,0x20,0x28,0x00,0x08,0x20,0xE8,0x3F,0x08,0x40,0x20,0x20,0x04,
  0x80,0x20,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* MOSFET enh p-ch */

  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x20,0x20,0x02,0x40,0x20,0x20,0x04,
  0x20,0xE8,0x3F,0x08,0x20,0x28,0x00,0x08,0x10,0x28,0x00,0x10,0x10,0x28,0x00,0x10,
  0x10,0x28,0x00,0x10,0x08,0x28,0x04,0x20,0x08,0x28,0x06,0x20,0x08,0xE8,0x3F,0x20,
  0x08,0x28,0x26,0x20,0x08,0x28,0x24,0x20,0x10,0x28,0x20,0x10,0x10,0x28,0x20,0x10,
  0x10,0x28,0x20,0x10,0x20,0x28,0x20,0x08,0xFF,0xEF,0x3F,0x08,0x40,0x20,0x20,0x04,
  0x80,0x20,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* MOSFET dep n-ch */

  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x20,0x20,0x02,0x40,0x20,0x20,0x04,
  0xFF,0xEF,0x3F,0x08,0x20,0x28,0x20,0x08,0x10,0x28,0x20,0x10,0x10,0x28,0x20,0x10,
  0x10,0x28,0x20,0x10,0x08,0x28,0x22,0x20,0x08,0x28,0x26,0x20,0x08,0xE8,0x3F,0x20,
  0x08,0x28,0x06,0x20,0x08,0x28,0x02,0x20,0x10,0x28,0x00,0x10,0x10,0x28,0x00,0x10,
  0x10,0x28,0x00,0x10,0x20,0x28,0x00,0x08,0x20,0xE8,0x3F,0x08,0x40,0x20,0x20,0x04,
  0x80,0x20,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* MOSFET dep p-ch */

  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x00,0x20,0x02,0x40,0x20,0x20,0x04,
  0x20,0x20,0x20,0x08,0x20,0xE0,0x3F,0x08,0x10,0x20,0x00,0x10,0x10,0x20,0x00,0x10,
  0x10,0x20,0x00,0x10,0x08,0x20,0x00,0x20,0x08,0x20,0x00,0x20,0x08,0x20,0x00,0x20,
  0x08,0x20,0x00,0x20,0x08,0x20,0x00,0x20,0x10,0x20,0x00,0x10,0x10,0x24,0x00,0x10,
  0x10,0x2C,0x00,0x10,0xFF,0xFF,0x3F,0x08,0x20,0x2C,0x20,0x08,0x40,0x24,0x20,0x04,
  0x80,0x00,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* JFET n-ch */

  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x00,0x20,0x02,0x40,0x24,0x20,0x04,
  0x20,0x26,0x20,0x08,0xFF,0xFF,0x3F,0x08,0x10,0x26,0x00,0x10,0x10,0x24,0x00,0x10,
  0x10,0x20,0x00,0x10,0x08,0x20,0x00,0x20,0x08,0x20,0x00,0x20,0x08,0x20,0x00,0x20,
  0x08,0x20,0x00,0x20,0x08,0x20,0x00,0x20,0x10,0x20,0x00,0x10,0x10,0x20,0x00,0x10,
  0x10,0x20,0x00,0x10,0x20,0xE0,0x3F,0x08,0x20,0x20,0x20,0x08,0x40,0x20,0x20,0x04,
  0x80,0x00,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* JFET p-ch */

  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x00,0x20,0x02,0x40,0x10,0x10,0x04,
  0x20,0x14,0x08,0x08,0x20,0x14,0x04,0x08,0x10,0x14,0x02,0x10,0x10,0x14,0x01,0x10,
  0x10,0x94,0x00,0x10,0x08,0x54,0x00,0x20,0x08,0x34,0x00,0x20,0xFF,0x17,0x00,0x20,
  0x08,0x34,0x00,0x20,0x08,0x54,0x00,0x20,0x10,0x94,0x00,0x10,0x10,0x14,0x09,0x10,
  0x10,0x14,0x0E,0x10,0x20,0x14,0x0E,0x08,0x20,0x14,0x0F,0x08,0x40,0x10,0x10,0x04,
  0x80,0x00,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* IGBT enh n-ch */

  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x00,0x20,0x02,0x40,0x00,0x10,0x04,
  0x20,0x10,0x09,0x08,0x20,0x14,0x07,0x08,0x10,0x14,0x07,0x10,0x10,0x14,0x0F,0x10,
  0x10,0x94,0x00,0x10,0x08,0x54,0x00,0x20,0x08,0x34,0x00,0x20,0xFF,0x17,0x00,0x20,
  0x08,0x34,0x00,0x20,0x08,0x54,0x00,0x20,0x10,0x94,0x00,0x10,0x10,0x14,0x01,0x10,
  0x10,0x14,0x02,0x10,0x20,0x14,0x04,0x08,0x20,0x10,0x08,0x08,0x40,0x00,0x10,0x04,
  0x80,0x00,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,   /* IGBT enh p-ch */

  0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,
  0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,
  0x00,0x00,0x01,0x00,0x80,0xFF,0xFF,0x03,0x00,0xFF,0xFF,0x01,0x00,0xFE,0xFF,0x00,
  0x00,0xFC,0x7F,0x00,0x00,0xF8,0x3F,0x00,0x00,0xF0,0x1F,0x00,0x00,0xE0,0x0F,0x00,
  0x00,0xC0,0x07,0x00,0x00,0x80,0x03,0x00,0x00,0x00,0x01,0x00,0x80,0xFF,0xFF,0x03,
  0x00,0x80,0x01,0x00,0x00,0x40,0x01,0x00,0x00,0x20,0x01,0x00,0x00,0x10,0x01,0x00,
  0x00,0x08,0x01,0x00,0x00,0x04,0x01,0x00,0xFE,0x03,0x01,0x00,0x00,0x00,0x01,0x00,
  0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,   /* SCR */

  0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,
  0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,
  0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0xC0,0xFF,0xFF,0x07,0x00,0x08,0xFE,0x03,
  0x00,0x1C,0xFE,0x03,0x00,0x1C,0xFC,0x01,0x00,0x3E,0xFC,0x01,0x00,0x3E,0xF8,0x00,
  0x00,0x7F,0xF8,0x00,0x00,0x7F,0x70,0x00,0x80,0xFF,0x70,0x00,0x80,0xFF,0x20,0x00,
  0xC0,0xFF,0xFF,0x07,0x00,0x08,0x01,0x00,0x00,0x04,0x01,0x00,0x00,0x02,0x01,0x00,
  0x00,0x01,0x01,0x00,0xFE,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,
  0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,   /* Triac */

  0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,
  0x00,0x00,0x01,0x00,0xFE,0x03,0x01,0x00,0x00,0x04,0x01,0x00,0x00,0x08,0x01,0x00,
  0x00,0x10,0x01,0x00,0x00,0x20,0x01,0x00,0x00,0x40,0x01,0x00,0x00,0x80,0x01,0x00,
  0x80,0xFF,0xFF,0x03,0x00,0xFF,0xFF,0x01,0x00,0xFE,0xFF,0x00,0x00,0xFC,0x7F,0x00,
  0x00,0xF8,0x3F,0x00,0x00,0xF0,0x1F,0x00,0x00,0xE0,0x0F,0x00,0x00,0xC0,0x07,0x00,
  0x00,0x80,0x03,0x00,0x00,0x00,0x01,0x00,0x80,0xFF,0xFF,0x03,0x00,0x00,0x01,0x00,
  0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,
  0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00    /* PUT */

  #ifdef SYMBOLS_EXTRA
  ,
  0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0xC0,0x27,0x00,0x00,0x38,0x38,0x00,
  0x00,0x06,0xE0,0x00,0x00,0x01,0x20,0x01,0x80,0x00,0x20,0x02,0x40,0x20,0x20,0x04,
  0x7F,0x20,0x20,0x08,0xA0,0xE0,0x3F,0x08,0x10,0x25,0x00,0x10,0x10,0x26,0x00,0x10,
  0x10,0x27,0x00,0x10,0x08,0x28,0x00,0x20,0x08,0x30,0x00,0x20,0x08,0x20,0x00,0x20,
  0x08,0x20,0x00,0x20,0x08,0x20,0x00,0x20,0x10,0x20,0x00,0x10,0x10,0x20,0x00,0x10,
  0x10,0x20,0x00,0x10,0x20,0xE0,0x3F,0x08,0x20,0x20,0x20,0x08,0x40,0x20,0x20,0x04,
  0x80,0x00,0x20,0x02,0x00,0x01,0x20,0x01,0x00,0x06,0xE0,0x00,0x00,0x38,0x38,0x00,
  0x00,0xC0,0x27,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00   /* UJT n-type */
  #endif
};



/*
 *  pin position lookup table
 *  - one byte per pin and 3 bytes (pins) for each symbol
 *  - cross reference for the Semi structure's pins:
 *
 *      BJT         FET         SCR         Triac       IGBT
 *  ------------------------------------------------------------------
 *  A   Base        Gate        Gate        Gate        Gate
 *  B   Collector   Drain       Anode       MT2         Collector
 *  C   Emitter     Source      Cathode     MT1         Emitter
 *
 *      PUT         UJT
 *  ------------------------------------------------------------------
 *  A   Gate        Emitter
 *  B   Anode       B2
 *  C   Cathode     B1
 */

const uint8_t PinTable[] PROGMEM = {
  PIN_LEFT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM,   /* BJT npn */
  PIN_LEFT | PIN_BOTTOM, PIN_RIGHT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP,   /* BJT pnp */
  PIN_LEFT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM,   /* MOSFET enh n-ch */
  PIN_LEFT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP,      /* MOSFET enh p-ch */
  PIN_LEFT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM,   /* MOSFET dep n-ch */
  PIN_LEFT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP,      /* MOSFET dep p-ch */
  PIN_LEFT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM,   /* JFET n-ch */
  PIN_LEFT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP,      /* JFET p-ch */
  PIN_LEFT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM,   /* IGBT enh n-ch */
  PIN_LEFT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP,      /* IGBT enh p-ch */
  PIN_LEFT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM,   /* SCR */
  PIN_LEFT | PIN_BOTTOM, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM,   /* Triac */
  PIN_LEFT | PIN_TOP, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM       /* PUT */
  #ifdef SYMBOLS_EXTRA
  ,
  PIN_LEFT | PIN_TOP, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM       /* UJT n-type */
  #endif
};



#endif

/* ************************************************************************
 *   EOF
 * ************************************************************************ */
