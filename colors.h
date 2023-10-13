/* ************************************************************************
 *
 *   color definitions for LCD modules
 *
 *   (c) 2015 by Markus Reschke
 *
 * ************************************************************************ */



/* ************************************************************************
 *   colors
 * ************************************************************************ */

/* ILI9341/ILI9342: 6 bit coded RGB */
#ifdef LCD_ILI9341_SPI
  #define COLOR_BLACK         0x0000
  #define COLOR_BLUE          0x001f
  #define COLOR_GREEN         0x07e0
  #define COLOR_CYAN          0x07ff
  #define COLOR_RED           0xf800
  #define COLOR_MAGENTA       0xf81f
  #define COLOR_YELLOW        0xffe0
  #define COLOR_ORANGE        0xfd20
  #define COLOR_WHITE         0xffff
#endif



/* ************************************************************************
 *   EOF
 * ************************************************************************ */
