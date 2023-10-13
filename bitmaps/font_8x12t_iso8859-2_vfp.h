/* ************************************************************************
 *
 *   thin monospaced 8x12 font based on ISO 8859-2
 *   vertically aligned, vertical bit order flipped, bank-wise grouping
 *
 *   provided by indman@EEVblog and Gennady_13@VRTP.RU
 *   font (c) by NickNi@VRTP.RU
 *   Central European characters added by Bohu
 *
 * ************************************************************************ */


/* ************************************************************************
 *   font data
 * ************************************************************************ */

#ifdef FONT_8X12T_ISO8859_2_VFP


/* font size */
#define FONT_SIZE_X          8     /* width:   8 dots */
#define FONT_SIZE_Y         12     /* heigth: 12 dots */

/* font data format */
#define FONT_BYTES_N        16     /* 16 bytes per character */
#define FONT_BYTES_X         8     /*  8 bytes in x direction */
#define FONT_BYTES_Y         2     /*  2 bytes in y direction */


/*
 *  character bitmaps
 *  - to reduce size we place some symbols and special characters at
 *    positions 0-15, and we move the standard chars up in the list by
 *    16 positions (using ASCII's 0-31 for control chars)
 *  - format:
 *    - 16 bytes per character
 *    - first byte: first vertical line (left to right)
 *    - ninth byte: second vertical line (left to right)
 *    - bit #0: top / bit #7: bottom (vertically flipped)
 *
 *  Example: character '2'
 *
 *  0x18,0x04,0x02,0x02,0x82,0x44,0x38,0x00,0x18,0x14,0x12,0x11,0x10,0x10,0x1C,0x00,	/ 0x22 '2' /
 * 
 *      18 04 02 02 82 44 38 00
 *  #0  -  -  -  -  -  -  -  -
 *      -  -  #  #  #  -  -  -
 *      -  #  -  -  -  #  -  -
 *      #  -  -  -  -  -  #  -
 *      #  -  -  -  -  -  #  -
 *      -  -  -  -  -  -  #  -
 *      -  -  -  -  -  #  -  -
 *  #7  -  -  -  -  #  -  -  -
 *
 *      18 14 12 11 10 10 1C 00
 *  #0  -  -  -  #  -  -  -  -
 *      -  -  #  -  -  -  -  -
 *      -  #  -  -  -  -  #  -
 *      #  -  -  -  -  -  #  -
 *      #  #  #  #  #  #  #  -
 *      -  -  -  -  -  -  -  -
 *      -  -  -  -  -  -  -  -
 *  #7  -  -  -  -  -  -  -  -
 *
 */

const uint8_t FontData[] PROGMEM = {
  /* symbols and special characters */ 
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x00 n/a */  
  0x40,0xFC,0xF8,0xF0,0xE0,0x40,0xFC,0x40,0x00,0x07,0x03,0x01,0x00,0x00,0x07,0x00,   /* 0x01 symbol: diode A-C */
  0x40,0xFC,0x40,0xE0,0xF0,0xF8,0xFC,0x40,0x00,0x07,0x00,0x00,0x01,0x03,0x07,0x00,   /* 0x02 symbol: diode C-A */
  0x40,0xFC,0xFC,0x00,0x00,0xFC,0xFC,0x40,0x00,0x07,0x07,0x00,0x00,0x07,0x07,0x00,   /* 0x03 symbol: capacitor */ 
  0x78,0x84,0x82,0x02,0x82,0x84,0x78,0x00,0x06,0x04,0x07,0x00,0x07,0x04,0x06,0x00,   /* 0x04 omega */
  0x00,0xFC,0x00,0x00,0x00,0xF0,0x10,0x00,0x10,0x0F,0x02,0x04,0x02,0x07,0x04,0x00,   /* 0x05 µ (micro) */
  0x08,0x08,0x08,0x08,0x08,0x08,0xF8,0x40,0x02,0x02,0x02,0x02,0x02,0x02,0x03,0x00,   /* 0x06 symbol: resistor left side */
  0x40,0xF8,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x03,0x02,0x02,0x02,0x02,0x02,0x02,   /* 0x07 symbol: resistor right side */

  0x00,0xE0,0x99,0x84,0x99,0xE0,0x00,0x04,0x07,0x04,0x00,0x04,0x04,0x07,0x04,0x00,   /* 0x08 Ä (A umlaut) */
  0xF0,0x08,0x05,0x04,0x05,0x08,0xF0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x09 Ö (O umlaut) */
  0x04,0xFC,0x05,0x00,0x05,0xFC,0x04,0x00,0x00,0x03,0x04,0x04,0x04,0x03,0x00,0x00,   /* 0x0a Ü (U umlaut) */
  0x00,0xF8,0x04,0x22,0x54,0x88,0x00,0x00,0x08,0x07,0x00,0x04,0x04,0x04,0x02,0x00,   /* 0x0b ß (sharp s) */
  0x90,0x48,0x2A,0x28,0x4A,0x90,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x07,0x00,   /* 0x0c ä (a umlaut) */
  0xE0,0x10,0x0A,0x08,0x0A,0x10,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x0d ö (o umlaut) */
  0x08,0xF8,0x0A,0x00,0x0A,0xF8,0x08,0x00,0x00,0x03,0x04,0x04,0x02,0x07,0x04,0x00,   /* 0x0e ü (u umlaut) */
  0x00,0x00,0x02,0x05,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x0f ° (degree sign) */

  /* standard characters */ 
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x10 ' ' */
  0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x00,0x00,0x00,0x00,   /* 0x11 '!' */
  0x00,0x16,0x0E,0x00,0x00,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x12 '"' */
  0x00,0x10,0xFC,0x10,0x10,0xFC,0x10,0x00,0x00,0x01,0x07,0x01,0x01,0x07,0x01,0x00,   /* 0x13 '#' */
  0x00,0x18,0x24,0xFE,0x44,0x88,0x00,0x00,0x00,0x03,0x04,0x0F,0x04,0x03,0x00,0x00,   /* 0x14 '$' */
  0x0C,0x12,0x92,0x4C,0x20,0x10,0x08,0x00,0x02,0x01,0x00,0x06,0x09,0x09,0x06,0x00,   /* 0x15 '%' */
  0x8C,0x52,0x22,0x52,0x8C,0x40,0xC0,0x00,0x01,0x02,0x04,0x04,0x04,0x01,0x02,0x00,   /* 0x16 '&' */
  0x00,0x00,0x00,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x17 ''' */
  0x00,0x00,0xF0,0x08,0x04,0x02,0x00,0x00,0x00,0x00,0x01,0x02,0x04,0x08,0x00,0x00,   /* 0x18 '(' */
  0x00,0x00,0x02,0x04,0x08,0xF0,0x00,0x00,0x00,0x00,0x08,0x04,0x02,0x01,0x00,0x00,   /* 0x19 ')' */
  0x48,0x50,0xE0,0xFC,0xE0,0x50,0x48,0x00,0x02,0x01,0x00,0x07,0x00,0x01,0x02,0x00,   /* 0x1a '*' */
  0x40,0x40,0x40,0xF8,0x40,0x40,0x40,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,   /* 0x1b '+' */
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x0C,0x00,0x00,0x00,   /* 0x1c ',' */
  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x1d '-' */
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00,0x00,0x00,   /* 0x1e '.' */
  0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x1f '/' */
  0xF8,0x04,0x02,0x02,0x02,0x04,0xF8,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x20 '0' */
  0x00,0x08,0x04,0xFE,0x02,0x00,0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,   /* 0x21 '1' */
  0x08,0x04,0x02,0x82,0x42,0x24,0x18,0x00,0x04,0x06,0x05,0x04,0x04,0x04,0x06,0x00,   /* 0x22 '2' */
  0x06,0x02,0x02,0x02,0x12,0x2A,0xC6,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x23 '3' */
  0xC0,0xA0,0x9E,0x80,0x80,0xFC,0x80,0x00,0x00,0x00,0x00,0x00,0x04,0x07,0x04,0x00,   /* 0x24 '4' */
  0x3E,0x12,0x0A,0x0A,0x0A,0x12,0xE6,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x25 '5' */
  0xF8,0x44,0x22,0x12,0x12,0x24,0xC0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x26 '6' */
  0x0E,0x02,0xC2,0x62,0x32,0x1A,0x0E,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,   /* 0x27 '7' */
  0x88,0x54,0x22,0x22,0x22,0x54,0x88,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x28 '8' */
  0x38,0x44,0x82,0x82,0x42,0x24,0xF8,0x00,0x00,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x29 '9' */
  0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x06,0x00,0x00,0x00,   /* 0x2a ':' */
  0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x16,0x0E,0x00,0x00,0x00,   /* 0x2b ';' */
  0x00,0x40,0xA0,0x10,0x08,0x04,0x02,0x00,0x00,0x00,0x00,0x01,0x02,0x04,0x08,0x00,   /* 0x2c '<' */
  0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x00,   /* 0x2d '=' */
  0x00,0x02,0x04,0x08,0x10,0xA0,0x40,0x00,0x00,0x08,0x04,0x02,0x01,0x00,0x00,0x00,   /* 0x2e '>' */
  0x18,0x04,0x02,0x82,0x42,0x24,0x18,0x00,0x00,0x00,0x00,0x0D,0x00,0x00,0x00,0x00,   /* 0x2f '?' */
  0xF8,0x04,0xD2,0x52,0xE2,0x04,0xF8,0x00,0x01,0x02,0x05,0x05,0x04,0x05,0x04,0x02,   /* 0x30 '@' */
  0x00,0xE0,0x98,0x84,0x98,0xE0,0x00,0x00,0x04,0x07,0x04,0x00,0x04,0x07,0x04,0x00,   /* 0x31 'A' */
  0x04,0xFC,0x44,0x44,0x44,0xA8,0x10,0x00,0x04,0x07,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x32 'B' */
  0xF0,0x08,0x04,0x04,0x04,0x08,0x9C,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x33 'C' */
  0x04,0xFC,0x04,0x04,0x04,0x08,0xF0,0x00,0x04,0x07,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x34 'D' */
  0x04,0xFC,0x44,0x44,0xE4,0x04,0x0C,0x00,0x04,0x07,0x04,0x04,0x04,0x04,0x07,0x00,   /* 0x35 'E' */
  0x04,0xFC,0x44,0x44,0xE4,0x04,0x0C,0x00,0x04,0x07,0x04,0x04,0x00,0x00,0x00,0x00,   /* 0x36 'F' */
  0xF0,0x08,0x04,0x84,0x84,0x88,0x9C,0x00,0x01,0x02,0x04,0x05,0x04,0x02,0x07,0x00,   /* 0x37 'G' */
  0x04,0xFC,0x44,0x40,0x44,0xFC,0x04,0x00,0x04,0x07,0x04,0x00,0x04,0x07,0x04,0x00,   /* 0x38 'H' */
  0x00,0x04,0x04,0xFC,0x04,0x04,0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,   /* 0x39 'I' */
  0x00,0x00,0x00,0x04,0x04,0xFC,0x04,0x00,0x00,0x03,0x04,0x04,0x04,0x03,0x00,0x00,   /* 0x3a 'J' */
  0x04,0xFC,0x44,0xC0,0x10,0x08,0x04,0x00,0x04,0x07,0x04,0x00,0x01,0x06,0x04,0x00,   /* 0x3b 'K' */
  0x04,0xFC,0x04,0x00,0x00,0x00,0x00,0x00,0x04,0x07,0x04,0x04,0x04,0x04,0x07,0x00,   /* 0x3c 'L' */
  0x04,0xFC,0x38,0xE0,0x38,0xFC,0x04,0x00,0x04,0x07,0x04,0x00,0x04,0x07,0x04,0x00,   /* 0x3d 'M' */
  0x04,0xFC,0x34,0x40,0x84,0xFC,0x04,0x00,0x04,0x07,0x04,0x00,0x05,0x07,0x04,0x00,   /* 0x3e 'N' */
  0xF0,0x08,0x04,0x04,0x04,0x08,0xF0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x3f 'O' */
  0x04,0xFC,0x44,0x44,0x44,0x28,0x10,0x00,0x04,0x07,0x04,0x00,0x00,0x00,0x00,0x00,   /* 0x40 'P' */
  0xF0,0x08,0x04,0x04,0x04,0x08,0xF0,0x00,0x01,0x02,0x04,0x04,0x04,0x05,0x02,0x05,   /* 0x41 'Q' */
  0x04,0xFC,0x44,0xC4,0x44,0x28,0x10,0x00,0x04,0x07,0x04,0x00,0x01,0x06,0x04,0x00,   /* 0x42 'R' */
  0x90,0x28,0x44,0x44,0x44,0x88,0x1C,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x43 'S' */
  0x0C,0x04,0x04,0xFC,0x04,0x04,0x0C,0x00,0x00,0x00,0x04,0x07,0x04,0x00,0x00,0x00,   /* 0x44 'T' */
  0x04,0xFC,0x04,0x00,0x04,0xFC,0x04,0x00,0x00,0x03,0x04,0x04,0x04,0x03,0x00,0x00,   /* 0x45 'U' */
  0x04,0x3C,0xC4,0x00,0xC4,0x3C,0x04,0x00,0x00,0x00,0x03,0x06,0x03,0x00,0x00,0x00,   /* 0x46 'V' */
  0x04,0xFC,0x04,0xE0,0x04,0xFC,0x04,0x00,0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00,   /* 0x47 'W' */
  0x04,0x1C,0xA4,0x40,0xA4,0x1C,0x04,0x00,0x04,0x07,0x04,0x00,0x04,0x07,0x04,0x00,   /* 0x48 'X' */
  0x04,0x1C,0x64,0x80,0x64,0x1C,0x04,0x00,0x00,0x00,0x04,0x07,0x04,0x00,0x00,0x00,   /* 0x49 'Y' */
  0x1C,0x04,0x84,0x44,0x24,0x14,0x0C,0x00,0x06,0x05,0x04,0x04,0x04,0x04,0x07,0x00,   /* 0x4a 'Z' */
  0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x07,0x04,0x04,0x04,0x00,0x00,0x00,   /* 0x4b '[' */
  0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x00,   /* 0x4c '\' */
  0x00,0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x07,0x00,0x00,   /* 0x4d ']' */
  0x10,0x08,0x04,0x02,0x04,0x08,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x4e '^' */
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,   /* 0x4f '_' */
  0x00,0x00,0x00,0x0E,0x16,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x50 '`' */
  0x90,0x48,0x28,0x28,0x48,0x90,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x07,0x00,   /* 0x51 'a' */
  0x02,0xFE,0x10,0x08,0x08,0x10,0xE0,0x00,0x04,0x07,0x02,0x04,0x04,0x02,0x01,0x00,   /* 0x52 'b' */
  0xE0,0x10,0x08,0x08,0x08,0x10,0x38,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x53 'c' */
  0xE0,0x10,0x08,0x08,0x10,0xFE,0x02,0x00,0x01,0x02,0x04,0x04,0x02,0x07,0x04,0x00,   /* 0x54 'd' */
  0xE0,0x50,0x48,0x48,0x48,0x50,0x60,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x55 'e' */
  0x20,0xF8,0x24,0x22,0x02,0x0C,0x00,0x00,0x04,0x07,0x04,0x04,0x00,0x00,0x00,0x00,   /* 0x56 'f' */
  0x60,0x90,0x08,0x08,0x08,0x90,0xF8,0x00,0x00,0x04,0x09,0x09,0x09,0x04,0x03,0x00,   /* 0x57 'g' */
  0x02,0xFE,0x10,0x08,0x08,0xF0,0x00,0x00,0x04,0x07,0x04,0x00,0x04,0x07,0x04,0x00,   /* 0x58 'h' */
  0x00,0x08,0x0A,0xFA,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,   /* 0x59 'i' */
  0x00,0x00,0x08,0x0A,0x0A,0xF8,0x00,0x00,0x00,0x00,0x04,0x08,0x08,0x07,0x00,0x00,   /* 0x5a 'j' */
  0x02,0xFE,0xC0,0x20,0x10,0x08,0x18,0x00,0x04,0x07,0x04,0x01,0x02,0x04,0x04,0x00,   /* 0x5b 'k' */
  0x00,0x00,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,   /* 0x5c 'l' */
  0x08,0xF8,0x08,0xF0,0x08,0xF0,0x00,0x00,0x04,0x07,0x00,0x07,0x00,0x07,0x04,0x00,   /* 0x5d 'm' */
  0x08,0xF8,0x10,0x08,0x10,0xE0,0x00,0x00,0x04,0x07,0x04,0x00,0x04,0x07,0x04,0x00,   /* 0x5e 'n' */
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x5f 'o' */
  0x08,0xF8,0x10,0x08,0x08,0x10,0xE0,0x00,0x08,0x0F,0x09,0x02,0x02,0x01,0x00,0x00,   /* 0x60 'p' */
  0xE0,0x10,0x08,0x08,0x10,0xF8,0x08,0x00,0x00,0x01,0x02,0x02,0x09,0x0F,0x08,0x00,   /* 0x61 'q' */
  0x08,0xF8,0x20,0x10,0x08,0x08,0x30,0x00,0x04,0x07,0x04,0x00,0x00,0x00,0x00,0x00,   /* 0x62 'r' */
  0x10,0x28,0x48,0x48,0x48,0x90,0x38,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x63 's' */
  0x08,0x08,0xFE,0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x03,0x04,0x04,0x04,0x02,0x00,   /* 0x64 't' */
  0x08,0xF8,0x08,0x00,0x08,0xF8,0x08,0x00,0x00,0x03,0x04,0x04,0x02,0x07,0x04,0x00,   /* 0x65 'u' */
  0x08,0xF8,0x88,0x00,0x88,0xF8,0x08,0x00,0x00,0x00,0x03,0x06,0x03,0x00,0x00,0x00,   /* 0x66 'v' */
  0x08,0xF8,0x08,0xC0,0x08,0xF8,0x08,0x00,0x00,0x01,0x06,0x01,0x06,0x01,0x00,0x00,   /* 0x67 'w' */
  0x08,0xB8,0xE0,0x40,0xE0,0xB8,0x08,0x00,0x04,0x07,0x00,0x00,0x00,0x07,0x04,0x00,   /* 0x68 'x' */
  0x08,0xF8,0x08,0x00,0x08,0xF8,0x08,0x00,0x00,0x04,0x09,0x09,0x08,0x07,0x00,0x00,   /* 0x69 'y' */
  0x18,0x08,0x88,0x48,0x28,0x18,0x08,0x00,0x06,0x05,0x04,0x04,0x04,0x04,0x06,0x00,   /* 0x6a 'z' */
  0x40,0xB0,0x1C,0x02,0x02,0x00,0x00,0x00,0x00,0x01,0x07,0x08,0x08,0x00,0x00,0x00,   /* 0x6b '{' */
  0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x00,0x00,0x00,0x00,   /* 0x6c '|' */
  0x00,0x00,0x02,0x02,0x1C,0xB0,0x40,0x00,0x00,0x00,0x08,0x08,0x07,0x01,0x00,0x00,   /* 0x6d '}' */
  0x0C,0x02,0x02,0x04,0x08,0x08,0x04,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x6e '~' */
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,   /* 0x6f n/a */

  /* Czech characters */
  0x00,0xE0,0x98,0x86,0x99,0xE0,0x00,0x04,0x07,0x04,0x00,0x04,0x04,0x07,0x04,0x00,   /* 0x6f Á (A with acute) */  
  0x90,0x48,0x28,0x2A,0x49,0x90,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x07,0x00,   /* 0x70 á (a with acute) */
  0x04,0xFC,0x44,0x46,0xE5,0x04,0x0C,0x00,0x04,0x07,0x04,0x04,0x04,0x04,0x07,0x00,   /* 0x71 É (E with acute) */
  0xE0,0x50,0x48,0x4A,0x49,0x50,0x60,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x72 é (e with acute) */
  0x00,0x04,0x04,0xFE,0x05,0x04,0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,   /* 0x73 Í (I with acute) */
  0x00,0x08,0x08,0xFA,0x01,0x00,0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,   /* 0x74 í (i with acute) */
  0xF0,0x08,0x04,0x06,0x05,0x08,0xF0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x75 Ó (O with acute) */
  0xE0,0x10,0x08,0x0A,0x09,0x10,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x76 ó (o with acute) */
  0x04,0xFC,0x04,0x02,0x05,0xFC,0x04,0x00,0x00,0x03,0x04,0x04,0x04,0x03,0x00,0x00,   /* 0x77 Ú (U with acute) */
  0x08,0xF8,0x08,0x02,0x09,0xF8,0x08,0x00,0x00,0x03,0x04,0x04,0x02,0x07,0x04,0x00,   /* 0x78 ú (u with acute) */
  0x04,0x1C,0x64,0x82,0x65,0x1C,0x04,0x00,0x00,0x00,0x04,0x07,0x04,0x00,0x00,0x00,   /* 0x79 Ý (Y with acute) */
  0x08,0xF8,0x08,0x02,0x89,0xF8,0x08,0x00,0x00,0x04,0x09,0x09,0x08,0x07,0x00,0x00,   /* 0x7a ý (y with acute) */
  0xF0,0x08,0x05,0x06,0x05,0x08,0x9C,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x7b Č (C with caron) */
  0xE0,0x10,0x09,0x0A,0x09,0x10,0x38,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x7c č (c with caron) */
  0x04,0xFC,0x05,0x06,0x05,0x08,0xF0,0x00,0x04,0x07,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x7d Ď (D with caron) */
  0xE0,0x10,0x09,0x0A,0x11,0xFE,0x02,0x00,0x01,0x02,0x04,0x04,0x02,0x07,0x04,0x00,   /* 0x7e d´ (d with caron) */
  0x04,0xFC,0x45,0x46,0xE5,0x04,0x0C,0x00,0x04,0x07,0x04,0x04,0x04,0x04,0x07,0x00,   /* 0x7f Ě (E with caron) */  
  0xE0,0x50,0x49,0x4A,0x49,0x50,0x60,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x80 ě (e with caron) */  
  0x04,0xFC,0x35,0x42,0x85,0xFC,0x04,0x00,0x04,0x07,0x04,0x00,0x05,0x07,0x04,0x00,   /* 0x81 Ň (N with caron) */
  0x08,0xF8,0x11,0x0A,0x11,0xE0,0x00,0x00,0x04,0x07,0x04,0x00,0x04,0x07,0x04,0x00,   /* 0x82 ň (n with caron) */
  0x04,0xFC,0x45,0xC6,0x45,0x28,0x10,0x00,0x04,0x07,0x04,0x00,0x01,0x06,0x04,0x00,   /* 0x83 Ř (R with caron) */
  0x08,0xF8,0x21,0x12,0x09,0x08,0x30,0x00,0x04,0x07,0x04,0x00,0x00,0x00,0x00,0x00,   /* 0x84 ř (r with caron) */
  0x90,0x28,0x45,0x46,0x45,0x88,0x1C,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x85 Š (S with caron) */
  0x10,0x28,0x49,0x4A,0x49,0x90,0x38,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x86 š (s with caron) */
  0x0C,0x04,0x05,0xFE,0x05,0x04,0x0C,0x00,0x00,0x00,0x04,0x07,0x04,0x00,0x00,0x00,   /* 0x87 Ť (T with caron) */
  0x08,0x08,0xFE,0x08,0x0B,0x00,0x00,0x00,0x00,0x00,0x03,0x04,0x04,0x04,0x02,0x00,   /* 0x88 t' (t with caron) */ 
  0x08,0xF8,0x02,0x05,0x02,0xF8,0x08,0x00,0x00,0x03,0x04,0x04,0x02,0x07,0x04,0x00,   /* 0x89 ů (u with ring above) */
  0x1C,0x04,0x85,0x46,0x25,0x14,0x0C,0x00,0x06,0x05,0x04,0x04,0x04,0x04,0x07,0x00,   /* 0x8a Ž (Z with caron) */
  0x18,0x08,0x89,0x4A,0x29,0x18,0x08,0x00,0x06,0x05,0x04,0x04,0x04,0x04,0x06,0x00,   /* 0x8b ž (z with caron) */

  /* additional Polish characters */
  0x00,0xE0,0x98,0x84,0x98,0xE0,0x00,0x00,0x04,0x07,0x04,0x00,0x04,0x0B,0x0A,0x00,   /* 0x8c Ą (A with ogonek) */  
  0x90,0x48,0x28,0x28,0x48,0x90,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x0A,0x0B,0x00,   /* 0x8d ą (a with ogonek) */
  0xF0,0x08,0x04,0x06,0x05,0x08,0x9C,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x8e Ć (C with caron) */
  0xE0,0x10,0x08,0x0A,0x09,0x10,0x38,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x8f ć (c with caron) */
  0x04,0xFC,0x44,0x44,0xE4,0x04,0x0C,0x00,0x04,0x07,0x04,0x04,0x04,0x0E,0x0A,0x00,   /* 0x90 Ę (E with ogonek) */
  0xE0,0x50,0x48,0x48,0x48,0x50,0x60,0x00,0x01,0x02,0x04,0x04,0x04,0x0A,0x0B,0x00,   /* 0x91 ę (e with ogonek) */
  0x44,0xFC,0x24,0x00,0x00,0x00,0x00,0x00,0x04,0x07,0x04,0x04,0x04,0x04,0x07,0x00,   /* 0x92 Ł (L with stroke) */
  0x00,0x00,0x02,0x42,0xFE,0x20,0x00,0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,   /* 0x93 ł (l with stroke) */
  0x04,0xFC,0x34,0x42,0x85,0xFC,0x04,0x00,0x04,0x07,0x04,0x00,0x05,0x07,0x04,0x00,   /* 0x94 Ń (N with acute) */
  0x08,0xF8,0x10,0x0A,0x11,0xE0,0x00,0x00,0x04,0x07,0x04,0x00,0x04,0x07,0x04,0x00,   /* 0x95 ń (n with acute) */
  0x90,0x28,0x44,0x46,0x45,0x88,0x1C,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x96 Ś (S with acute) */
  0x10,0x28,0x48,0x4A,0x49,0x90,0x38,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0x97 ś (s with acute) */
  0x1C,0x04,0x84,0x46,0x25,0x14,0x0C,0x00,0x06,0x05,0x04,0x04,0x04,0x04,0x07,0x00,   /* 0x98 Ź (Z with acute) */
  0x18,0x08,0x88,0x4A,0x29,0x18,0x08,0x00,0x06,0x05,0x04,0x04,0x04,0x04,0x06,0x00,   /* 0x99 ź (z with acute) */
  0x1C,0x04,0x84,0x46,0x24,0x14,0x0C,0x00,0x06,0x05,0x04,0x04,0x04,0x04,0x07,0x00,   /* 0x9a Ż (Z with dot above) */
  0x18,0x08,0x88,0x4A,0x28,0x18,0x08,0x00,0x06,0x05,0x04,0x04,0x04,0x04,0x06,0x00,   /* 0x9b ż (z with dot above) */

  /* additional Romanian characters */
  0x00,0xE0,0x9A,0x85,0x9A,0xE0,0x00,0x04,0x07,0x04,0x00,0x04,0x04,0x07,0x04,0x00,   /* 0x9c Â (A with circumflex) */  
  0x90,0x48,0x2A,0x29,0x4A,0x90,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x07,0x00,   /* 0x9d â (a with circumflex) */
  0x00,0xE0,0x99,0x86,0x99,0xE0,0x00,0x04,0x07,0x04,0x00,0x04,0x04,0x07,0x04,0x00,   /* 0x9e Ă (A with breve) */  
  0x90,0x48,0x29,0x2A,0x49,0x90,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x07,0x00,   /* 0x9f ă (a with breve) */
  0x00,0x04,0x06,0xFD,0x06,0x04,0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,   /* 0xa0 Î (I with circumflex) */
  0x00,0x08,0x0A,0xF9,0x02,0x00,0x00,0x00,0x00,0x04,0x04,0x07,0x04,0x04,0x00,0x00,   /* 0xa1 î (i with circumflex) */
  0x90,0x28,0x44,0x44,0x44,0x88,0x1C,0x00,0x01,0x02,0x04,0x04,0x04,0x0A,0x0B,0x00,   /* 0xa2 Ş (S with cedilla) */
  0x10,0x28,0x48,0x48,0x48,0x90,0x38,0x00,0x01,0x02,0x04,0x04,0x04,0x0A,0x0B,0x00,   /* 0xa3 ş (s with cedilla) */
  0x0C,0x04,0x04,0xFC,0x04,0x04,0x0C,0x00,0x00,0x00,0x0A,0x0B,0x04,0x00,0x00,0x00,   /* 0xa4 Ţ (T with cedilla) */
  0x08,0x08,0xFE,0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x0B,0x0A,0x04,0x04,0x02,0x00,   /* 0xa5 ţ (t with cedilla) */ 

  /* additional Hungarian characters */
  0xF0,0x08,0x07,0x04,0x07,0x08,0xF0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0xa6 Ő (O with double acute) */
  0xE0,0x10,0x0B,0x08,0x0B,0x10,0xE0,0x00,0x01,0x02,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0xa7 ő (o with double acute) */
  0x04,0xF8,0x03,0x00,0x03,0xF8,0x04,0x00,0x00,0x03,0x04,0x04,0x04,0x03,0x00,0x00,   /* 0xa8 Ű (U with double acute) */
  0x08,0xF8,0x0B,0x00,0x0B,0xF8,0x08,0x00,0x00,0x03,0x04,0x04,0x02,0x07,0x04,0x00,   /* 0xa9 ű (u with double acute) */

  /* additional Serbo-Croatian characters */
  0x44,0xFC,0x44,0x04,0x04,0x08,0xF0,0x00,0x04,0x07,0x04,0x04,0x04,0x02,0x01,0x00,   /* 0xaa Đ (D with stroke) */
  0xE0,0x10,0x08,0x08,0x12,0xFF,0x02,0x00,0x01,0x02,0x04,0x04,0x02,0x07,0x04,0x00,   /* 0xab đ (d with stroke) */

  /* additional Albanian characters */
  0xF0,0x08,0x04,0x04,0x04,0x08,0x9C,0x00,0x01,0x02,0x04,0x04,0x04,0x0B,0x0A,0x00,   /* 0xac Ç (C with cedilla) */
  0xE0,0x10,0x08,0x08,0x08,0x10,0x38,0x00,0x01,0x02,0x04,0x04,0x04,0x0A,0x0B,0x00    /* 0xad ç (c with cedilla) */
};


/*
 *  font lookup table for ISO 8859-2
 *  - 8 bit ISO 8859-2 to custom font
 *  - ff: no bitmap available
 */

const uint8_t FontTable[] PROGMEM = {
  0xff,        /* 0x00 -> n/a */
  0x01,        /* 0x01 -> symbol: diode A-C */
  0x02,        /* 0x02 -> symbol: diode C-A */
  0x03,        /* 0x03 -> symbol: capacitor */
  0x04,        /* 0x04 -> omega */
  0xff,        /* 0x05 -> b5 */
  0x06,        /* 0x06 -> symbol: resistor left side */
  0x07,        /* 0x07 -> symbol: resistor right side */
  0xff,        /* 0x08 -> c4 */
  0xff,        /* 0x09 -> d6 */
  0xff,        /* 0x0a -> dc */
  0xff,        /* 0x0b -> df */
  0xff,        /* 0x0c -> e4 */
  0xff,        /* 0x0d -> f6 */
  0xff,        /* 0x0e -> fc */
  0xff,        /* 0x0f -> b0 */

  0xff,        /* 0x10 -> n/a */
  0xff,        /* 0x11 -> n/a */
  0xff,        /* 0x12 -> n/a */
  0xff,        /* 0x13 -> n/a */
  0xff,        /* 0x14 -> n/a */
  0xff,        /* 0x15 -> n/a */
  0xff,        /* 0x16 -> n/a */
  0xff,        /* 0x17 -> n/a */
  0xff,        /* 0x18 -> n/a */
  0xff,        /* 0x19 -> n/a */
  0xff,        /* 0x1a -> n/a */
  0xff,        /* 0x1b -> n/a */
  0xff,        /* 0x1c -> n/a */
  0xff,        /* 0x1d -> n/a */
  0xff,        /* 0x1e -> n/a */
  0xff,        /* 0x1f -> n/a */

  0x10,        /* 0x20 -> space */
  0x11,        /* 0x21 -> ! */
  0x12,        /* 0x22 -> " */
  0x13,        /* 0x23 -> # */
  0x14,        /* 0x24 -> $ */
  0x15,        /* 0x25 -> % */
  0x16,        /* 0x26 -> & */
  0x17,        /* 0x27 -> ´ */
  0x18,        /* 0x28 -> ( */
  0x19,        /* 0x29 -> ) */
  0x1a,        /* 0x2a -> * */
  0x1b,        /* 0x2b -> + */
  0x1c,        /* 0x2c -> , */
  0x1d,        /* 0x2d -> - */
  0x1e,        /* 0x2e -> . */
  0x1f,        /* 0x2f -> / */

  0x20,        /* 0x30 -> 0 */
  0x21,        /* 0x31 -> 1 */
  0x22,        /* 0x32 -> 2 */
  0x23,        /* 0x33 -> 3 */
  0x24,        /* 0x34 -> 4 */
  0x25,        /* 0x35 -> 5 */
  0x26,        /* 0x36 -> 6 */
  0x27,        /* 0x37 -> 7 */
  0x28,        /* 0x38 -> 8 */
  0x29,        /* 0x39 -> 9 */
  0x2a,        /* 0x3a -> : */
  0x2b,        /* 0x3b -> ; */
  0x2c,        /* 0x3c -> < */
  0x2d,        /* 0x3d -> = */
  0x2e,        /* 0x3e -> > */
  0x2f,        /* 0x3f -> ? */

  0x30,        /* 0x40 -> @ */
  0x31,        /* 0x41 -> A */
  0x32,        /* 0x42 -> B */
  0x33,        /* 0x43 -> C */
  0x34,        /* 0x44 -> D */
  0x35,        /* 0x45 -> E */
  0x36,        /* 0x46 -> F */
  0x37,        /* 0x47 -> G */
  0x38,        /* 0x48 -> H */
  0x39,        /* 0x49 -> I */
  0x3a,        /* 0x4a -> J */
  0x3b,        /* 0x4b -> K */
  0x3c,        /* 0x4c -> L */
  0x3d,        /* 0x4d -> M */
  0x3e,        /* 0x4e -> N */
  0x3f,        /* 0x4f -> O */

  0x40,        /* 0x50 -> P */
  0x41,        /* 0x51 -> Q */
  0x42,        /* 0x52 -> R */
  0x43,        /* 0x53 -> S */
  0x44,        /* 0x54 -> T */
  0x45,        /* 0x55 -> U */
  0x46,        /* 0x56 -> V */
  0x47,        /* 0x57 -> W */
  0x48,        /* 0x58 -> X */
  0x49,        /* 0x59 -> Y */
  0x4a,        /* 0x5a -> Z */
  0x4b,        /* 0x5b -> [ */
  0x4c,        /* 0x5c -> \ */
  0x4d,        /* 0x5d -> ] */
  0x4e,        /* 0x5e -> ^ */
  0x4f,        /* 0x5f -> _ */

  0x50,        /* 0x60 -> ` */
  0x51,        /* 0x61 -> a */
  0x52,        /* 0x62 -> b */
  0x53,        /* 0x63 -> c */
  0x54,        /* 0x64 -> d */
  0x55,        /* 0x65 -> e */
  0x56,        /* 0x66 -> f */
  0x57,        /* 0x67 -> g */
  0x58,        /* 0x68 -> h */
  0x59,        /* 0x69 -> i */
  0x5a,        /* 0x6a -> j */
  0x5b,        /* 0x6b -> k */
  0x5c,        /* 0x6c -> l */
  0x5d,        /* 0x6d -> m */
  0x5e,        /* 0x6e -> n */
  0x5f,        /* 0x6f -> o */

  0x60,        /* 0x70 -> p */
  0x61,        /* 0x71 -> q */
  0x62,        /* 0x72 -> r */
  0x63,        /* 0x73 -> s */
  0x64,        /* 0x74 -> t */
  0x65,        /* 0x75 -> u */
  0x66,        /* 0x76 -> v */
  0x67,        /* 0x77 -> w */
  0x68,        /* 0x78 -> x */
  0x69,        /* 0x79 -> y */
  0x6a,        /* 0x7a -> z */
  0x6b,        /* 0x7b -> { */
  0x6c,        /* 0x7c -> | */
  0x6d,        /* 0x7d -> } */
  0x6e,        /* 0x7e -> ~ */
  0xff,        /* 0x7f -> n/a */

  0xff,        /* 0x80 -> n/a */
  0xff,        /* 0x81 -> n/a */
  0xff,        /* 0x82 -> n/a */
  0xff,        /* 0x83 -> n/a */
  0xff,        /* 0x84 -> n/a */
  0xff,        /* 0x85 -> n/a */
  0xff,        /* 0x86 -> n/a */
  0xff,        /* 0x87 -> n/a */
  0xff,        /* 0x88 -> n/a */
  0xff,        /* 0x89 -> n/a */
  0xff,        /* 0x8a -> n/a */
  0xff,        /* 0x8b -> n/a */
  0xff,        /* 0x8c -> n/a */
  0xff,        /* 0x8d -> n/a */
  0xff,        /* 0x8e -> n/a */
  0xff,        /* 0x8f -> n/a */

  0xff,        /* 0x90 -> n/a */
  0xff,        /* 0x91 -> n/a */
  0xff,        /* 0x92 -> n/a */
  0xff,        /* 0x93 -> n/a */
  0xff,        /* 0x94 -> n/a */
  0xff,        /* 0x95 -> n/a */
  0xff,        /* 0x96 -> n/a */
  0xff,        /* 0x97 -> n/a */
  0xff,        /* 0x98 -> n/a */
  0xff,        /* 0x99 -> n/a */
  0xff,        /* 0x9a -> n/a */
  0xff,        /* 0x9b -> n/a */
  0xff,        /* 0x9c -> n/a */
  0xff,        /* 0x9d -> n/a */
  0xff,        /* 0x9e -> n/a */
  0xff,        /* 0x9f -> n/a */

  0xff,        /* 0xa0 -> n/a */
  0x8c,        /* 0xa1 -> Ą (A with ogonek) */
  0xff,        /* 0xa2 -> n/a */
  0x92,        /* 0xa3 -> Ł (L with stroke) */
  0xff,        /* 0xa4 -> n/a */
  0xff,        /* 0xa5 -> n/a */
  0x96,        /* 0xa6 -> Ś (S with acute) */
  0xff,        /* 0xa7 -> n/a */
  0xff,        /* 0xa8 -> n/a */
  0x85,        /* 0xa9 -> Š (S with caron) */
  0xa2,        /* 0xaa -> Ş (S with cedilla) */
  0x87,        /* 0xab -> Ť (T with caron) */
  0x98,        /* 0xac -> Ź (Z with acute) */
  0xff,        /* 0xad -> n/a */
  0x8a,        /* 0xae -> Ž (Z with caron) */
  0x9a,        /* 0xaf -> Ż (Z with dot above) */

  0x0f,        /* 0xb0 -> ° (degree sign) */
  0x8d,        /* 0xb1 -> ą (a with ogonek) */  
  0xff,        /* 0xb2 -> n/a */
  0x93,        /* 0xb3 -> ł (l with stroke) */
  0xff,        /* 0xb4 -> n/a */
  0x05,        /* 0xb5 -> µ (micro) */
  0x97,        /* 0xb6 -> ś (s with acute) */
  0xff,        /* 0xb7 -> n/a */
  0xff,        /* 0xb8 -> n/a */
  0x86,        /* 0xb9 -> š (s with caron) */
  0xa3,        /* 0xba -> ş (s with cedilla) */
  0x88,        /* 0xbb -> t' (t with caron) */
  0x99,        /* 0xbc -> ź (z with acute) */
  0xff,        /* 0xbd -> n/a */
  0x8b,        /* 0xbe -> ž (z with caron) */
  0x9b,        /* 0xbf -> ż (z with dot above) */
  
  0xff,        /* 0xc0 -> n/a */
  0x6f,        /* 0xc1 -> Á (A with acute) */
  0x9c,        /* 0xc2 -> Â (A with circumflex) */  
  0x9e,        /* 0xc3 -> Ă (A with breve) */ 
  0x08,        /* 0xc4 -> Ä (A umlaut) */
  0xff,        /* 0xc5 -> n/a */
  0x8e,        /* 0xc6 -> Ć (C with caron) */
  0xac,        /* 0xc7 -> Ç (C with cedilla) */
  0x7b,        /* 0xc8 -> Č (C with caron) */
  0x71,        /* 0xc9 -> É (E with acute) */
  0x90,        /* 0xca -> Ę (E with ogonek) */
  0xff,        /* 0xcb -> n/a */
  0x7f,        /* 0xcc -> Ě (E with caron) */
  0x73,        /* 0xcd -> Í (I with acute) */
  0xa0,        /* 0xce -> Î (I with circumflex) */
  0x7d,        /* 0xcf -> Ď (D with caron) */

  0xaa,        /* 0xd0 -> Đ (D with stroke) */
  0x94,        /* 0xd1 -> Ń (N with acute) */
  0x81,        /* 0xd2 -> Ň (N with caron) */
  0x75,        /* 0xd3 -> Ó (O with acute) */
  0xff,        /* 0xd4 -> n/a */
  0xa6,        /* 0xd5 -> Ő (O with double acute) */
  0x09,        /* 0xd6 -> Ö (O umlaut) */
  0xff,        /* 0xd7 -> n/a */
  0x83,        /* 0xd8 -> Ř (R with caron) */
  0xff,        /* 0xd9 -> n/a */
  0x77,        /* 0xda -> Ú (U with acute) */
  0xa8,        /* 0xdb -> Ű (U with double acute) */
  0x0a,        /* 0xdc -> Ü (U umlaut) */
  0x79,        /* 0xdd -> Ý (Y with acute) */
  0xa4,        /* 0xde -> Ţ (T with cedilla) */
  0x0b,        /* 0xdf -> ß (sharp s) */
  
  0xff,        /* 0xe0 -> n/a */
  0x70,        /* 0xe1 -> á (a with acute) */
  0x9d,        /* 0xe2 -> â (a with circumflex) */
  0x9f,        /* 0xe3 -> ă (a with breve) */
  0x0c,        /* 0xe4 -> ä (a umlaut) */
  0xff,        /* 0xe5 -> n/a */
  0x8f,        /* 0xe6 -> ć (c with caron) */
  0xad,        /* 0xe7 -> ç (c with cedilla) */
  0x7c,        /* 0xe8 -> č (c with caron) */
  0x72,        /* 0xe9 -> é (e with acute) */
  0x91,        /* 0xea -> ę (e with ogonek) */
  0xff,        /* 0xeb -> n/a */
  0x80,        /* 0xec -> ě (e with caron) */
  0x74,        /* 0xed -> í (i with acute) */
  0xa1,        /* 0xee -> î (i with circumflex) */
  0x7e,        /* 0xef -> d´ (d with caron) */

  0xab,        /* 0xf0 -> đ (d with stroke) */
  0x95,        /* 0xf1 -> ń (n with acute) */
  0x82,        /* 0xf2 -> ň (n with caron) */
  0x76,        /* 0xf3 -> ó (o with acute)*/
  0xff,        /* 0xf4 -> n/a */
  0xa7,        /* 0xf5 -> ő (o with double acute) */
  0x0d,        /* 0xf6 -> ö (o umlaut) */
  0xff,        /* 0xf7 -> n/a */
  0x84,        /* 0xf8 -> ř (r with caron) */
  0x89,        /* 0xf9 -> ů (u with ring above) */
  0x78,        /* 0xfa -> ú (u with acute)*/
  0xa9,        /* 0xfb -> ű (u with double acute) */
  0x0e,        /* 0xfc -> ü (u umlaut) */
  0x7a,        /* 0xfd -> ý (y with acute)*/
  0xa5,        /* 0xfe -> ţ (t with cedilla) */ 
  0xff         /* 0xff -> n/a */
};


#endif

/* ************************************************************************
 *   EOF
 * ************************************************************************ */                            
