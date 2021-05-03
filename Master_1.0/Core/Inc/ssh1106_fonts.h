#include <stdint.h>

#ifndef __SSH1106_FONTS_H__
#define __SSH1106_FONTS_H__

#include <ssh1106_conf.h>

typedef struct {
	const uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef;

#ifdef SSH1106_INCLUDE_FONT_6x8
extern FontDef Font_6x8;
#endif
#ifdef SSH1106_INCLUDE_FONT_7x10
extern FontDef Font_7x10;
#endif
#ifdef SSH1106_INCLUDE_FONT_11x18
extern FontDef Font_11x18;
#endif
#ifdef SSH1106_INCLUDE_FONT_16x26
extern FontDef Font_16x26;
#endif
#endif // __SSH1106_FONTS_H__
