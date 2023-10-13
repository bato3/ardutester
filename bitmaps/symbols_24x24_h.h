/* ************************************************************************
 *
 *   24x24 component symbols 
 *   horizontally aligned
 *
 *   (c) 2015-2016 by Markus Reschke
 *
 * ************************************************************************ */


/* ************************************************************************
 *   symbol data
 * ************************************************************************ */

#ifdef SYMBOLS_24X24_H


/* symbol size */
#define SYMBOL_SIZE_X       24     /* width:  24 dots */
#define SYMBOL_SIZE_Y       24     /* heigth: 24 dots */

/* symbol data format */
#define SYMBOL_BYTES_N      72     /* 72 bytes per symbol */
#define SYMBOL_BYTES_X       3     /* 3 bytes in x direction */
#define SYMBOL_BYTES_Y      24     /* 24 bytes in y direction */


/*
 *  symbol bitmaps
 *  - format:
 *    - 72 bytes per symbol
 *    - first three bytes: first horizontal line (top to down)
 *      first byte: left part
 *    - bit #0: left / bit #7: right (horizontally flipped)
 */
/* todo: this is vertically aligned and needs conversion */
const uint8_t SymbolData[] PROGMEM = {
  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x00,0x09,0x20,0x86,0x08,0x10,0x46,0x10,0x10,0x26,0x10,0x08,0x16,0x20,0x08,0x0E,
  0x20,0xFF,0x07,0x20,0x08,0x0E,0x20,0x08,0x16,0x20,0x10,0xA6,0x10,0x10,0xC6,0x10,
  0x20,0xE6,0x08,0x20,0x00,0x09,0xC0,0x00,0x06,0x00,0x83,0x03,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* BJT npn */

  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x00,0x09,0x20,0xA6,0x08,0x10,0x66,0x10,0x10,0xE6,0x10,0x08,0x16,0x20,0x08,0x0E,
  0x20,0xFF,0x07,0x20,0x08,0x0E,0x20,0x08,0x16,0x20,0x10,0x26,0x10,0x10,0x46,0x10,
  0x20,0x86,0x08,0x20,0x00,0x09,0xC0,0x00,0x06,0x00,0x83,0x03,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* BJT pnp */

  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x04,0x0A,0x20,0xFD,0x0B,0x10,0x05,0x10,0x10,0x01,0x10,0x08,0x41,0x20,0x08,0x65,
  0x20,0x08,0xFD,0x23,0x08,0x65,0x22,0x08,0x41,0x22,0x10,0x01,0x12,0x10,0x05,0x12,
  0xFF,0xFD,0x0B,0x20,0x04,0x0A,0xC0,0x00,0x06,0x00,0x83,0x07,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* MOSFET enh n-ch */

  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x04,0x0A,0xFF,0xFD,0x0B,0x10,0x05,0x10,0x10,0x01,0x10,0x08,0x21,0x20,0x08,0x65,
  0x20,0x08,0xFD,0x23,0x08,0x65,0x22,0x08,0x21,0x22,0x10,0x01,0x12,0x10,0x05,0x12,
  0x20,0xFD,0x0B,0x20,0x04,0x0A,0xC0,0x00,0x06,0x00,0x83,0x03,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* MOSFET enh p-ch */

  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x04,0x0A,0x20,0xFD,0x0B,0x10,0x05,0x10,0x10,0x05,0x10,0x08,0x45,0x20,0x08,0x65,
  0x20,0x08,0xFD,0x23,0x08,0x65,0x22,0x08,0x45,0x22,0x10,0x05,0x12,0x10,0x05,0x12,
  0xFF,0xFD,0x0B,0x20,0x04,0x0A,0xC0,0x00,0x06,0x00,0x83,0x07,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* MOSFET dep n-ch */

  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x04,0x0A,0xFF,0xFD,0x0B,0x10,0x05,0x10,0x10,0x05,0x10,0x08,0x25,0x20,0x08,0x65,
  0x20,0x08,0xFD,0x23,0x08,0x65,0x22,0x08,0x25,0x22,0x10,0x05,0x12,0x10,0x05,0x12,
  0x20,0xFD,0x0B,0x20,0x04,0x0A,0xC0,0x00,0x06,0x00,0x83,0x03,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* MOSFET dep p-ch */

  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x08,0x0A,0x20,0x08,0x0A,0x10,0xF8,0x13,0x10,0x08,0x10,0x08,0x08,0x20,0x08,0x08,
  0x20,0x08,0x08,0x20,0x08,0x08,0x20,0x08,0x09,0x20,0x10,0x0B,0x10,0xFF,0xFF,0x13,
  0x20,0x0B,0x0A,0x20,0x09,0x0A,0xC0,0x00,0x06,0x00,0x83,0x03,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* JFET n-ch */

  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x0A,0x0A,0x20,0x0B,0x0A,0xFF,0xFF,0x13,0x10,0x0B,0x10,0x08,0x0A,0x20,0x08,0x08,
  0x20,0x08,0x08,0x20,0x08,0x08,0x20,0x08,0x08,0x20,0x10,0x08,0x10,0x10,0xF8,0x13,
  0x20,0x08,0x0A,0x20,0x08,0x0A,0xC0,0x00,0x06,0x00,0x83,0x03,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* JFET p-ch */

  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x00,0x09,0x20,0x84,0x08,0x10,0x45,0x10,0x10,0x25,0x10,0x08,0x15,0x20,0x08,0x0D,
  0x20,0xFF,0x05,0x20,0x08,0x0D,0x20,0x08,0x15,0x20,0x10,0xA5,0x10,0x10,0xC5,0x10,
  0x20,0xE4,0x08,0x20,0x00,0x09,0xC0,0x00,0x06,0x00,0x83,0x03,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* IGBT enh n-ch */

  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x00,0x09,0x20,0xA4,0x08,0x10,0x65,0x10,0x10,0xE5,0x10,0x08,0x15,0x20,0x08,0x0D,
  0x20,0xFF,0x05,0x20,0x08,0x0D,0x20,0x08,0x15,0x20,0x10,0x25,0x10,0x10,0x45,0x10,
  0x20,0x84,0x08,0x20,0x00,0x09,0xC0,0x00,0x06,0x00,0x83,0x03,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00,   /* IGBT enh p-ch */

  0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x80,
  0xFF,0x03,0x00,0xFF,0x01,0x00,0xFF,0x01,0x00,0xFE,0x00,0x00,0xFE,0x00,0x00,0x7C,
  0x00,0x00,0x7C,0x00,0x00,0x38,0x00,0x00,0x38,0x00,0x00,0x10,0x00,0x80,0xFF,0x03,
  0x00,0x18,0x00,0x00,0x14,0x00,0x00,0x12,0x00,0xFE,0x11,0x00,0x00,0x10,0x00,0x00,
  0x10,0x00,0x00,0x10,0x00,0x00,0x00,0x00,   /* SCR */

  0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,
  0x10,0x00,0x00,0x10,0x00,0xF8,0xFF,0x3F,0x80,0xE0,0x3F,0xC0,0xC1,0x1F,0xC0,0xC1,
  0x1F,0xE0,0x83,0x0F,0xE0,0x83,0x0F,0xF0,0x07,0x07,0xF0,0x07,0x07,0xF8,0x0F,0x02,
  0xF8,0xFF,0x3F,0x80,0x10,0x00,0x40,0x10,0x00,0x20,0x10,0x00,0x1F,0x10,0x00,0x00,
  0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,   /* Triac */

  0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0x00,0x10,0x00,0xFE,0x11,0x00,0x00,
  0x12,0x00,0x00,0x14,0x00,0x00,0x18,0x00,0x80,0xFF,0x03,0x00,0xFF,0x01,0x00,0xFF,
  0x01,0x00,0xFE,0x00,0x00,0xFE,0x00,0x00,0x7C,0x00,0x00,0x7C,0x00,0x00,0x38,0x00,
  0x00,0x38,0x00,0x00,0x10,0x00,0x80,0xFF,0x03,0x00,0x10,0x00,0x00,0x10,0x00,0x00,
  0x10,0x00,0x00,0x10,0x00,0x00,0x00,0x00    /* PUT */

  #ifdef SW_UJT
  ,
  0x00,0x00,0x02,0x00,0x00,0x02,0x00,0x7C,0x02,0x00,0x83,0x03,0xC0,0x00,0x06,0x20,
  0x08,0x0A,0x3F,0x08,0x0A,0x50,0xF8,0x13,0x90,0x0A,0x10,0x08,0x0B,0x20,0x88,0x0B,
  0x20,0x08,0x0C,0x20,0x08,0x08,0x20,0x08,0x08,0x20,0x10,0x08,0x10,0x10,0xF8,0x13,
  0x20,0x08,0x0A,0x20,0x08,0x0A,0xC0,0x00,0x06,0x00,0x83,0x03,0x00,0x7C,0x02,0x00,
  0x00,0x02,0x00,0x00,0x02,0x00,0x00,0x00    /* UJT */
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
  #ifdef SW_UJT
  ,
  PIN_LEFT | PIN_TOP, PIN_RIGHT | PIN_TOP, PIN_RIGHT | PIN_BOTTOM       /* UJT */
  #endif
};



#endif

/* ************************************************************************
 *   EOF
 * ************************************************************************ */
