#ifndef __SSH1106_CONF_H__
#define __SSH1106_CONF_H__

// Choose a microcontroller family
//#define STM32F0
//#define STM32F1
#define STM32F4
//#define STM32L0
//#define STM32L4
//#define STM32F3
//#define STM32H7
//#define STM32F7

// Choose a bus
#define SSH1106_USE_I2C
//#define SSH1106_USE_SPI

// I2C Configuration
#define SSH1106_I2C_PORT        hi2c1
#define SSH1106_I2C_ADDR        (0x3C << 1)

// SPI Configuration
//#define SSH1106_SPI_PORT        hspi1
//#define SSH1106_CS_Port         OLED_CS_GPIO_Port
//#define SSH1106_CS_Pin          OLED_CS_Pin
//#define SSH1106_DC_Port         OLED_DC_GPIO_Port
//#define SSH1106_DC_Pin          OLED_DC_Pin
//#define SSH1106_Reset_Port      OLED_Res_GPIO_Port
//#define SSH1106_Reset_Pin       OLED_Res_Pin

// Mirror the screen if needed
// #define SSH1106_MIRROR_VERT
// #define SSH1106_MIRROR_HORIZ

// Set inverse color if needed
// # define SSH1106_INVERSE_COLOR

// Include only needed fonts
#define SSH1106_INCLUDE_FONT_6x8
#define SSH1106_INCLUDE_FONT_7x10
#define SSH1106_INCLUDE_FONT_11x18
#define SSH1106_INCLUDE_FONT_16x26

// Some OLEDs don't display anything in first two columns.
// In this case change the following macro to 130.
// The default value is 128.
// #define SSH1106_WIDTH           130

// The height can be changed as well if necessary.
// It can be 32, 64 or 128. The default value is 64.
// #define SSH1106_HEIGHT          64

#endif /* __SSH1106_CONF_H__ */
