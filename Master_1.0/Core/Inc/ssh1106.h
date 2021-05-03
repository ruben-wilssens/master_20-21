/**
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 * https://github.com/afiskon/stm32-ssh1106
 *
 *Adapted by Victor Van der Elst for Master Thesis 2021
 */

#ifndef INC_SSH1106_H_
#define INC_SSH1106_H_

#include <stddef.h>
#include <_ansi.h>

_BEGIN_STD_C

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "ssh1106_fonts.h"

/* vvv I2C config vvv */
#define SSH1106_I2C_PORT        hi2c1
#define SSH1106_I2C_ADDR        (0x3C << 1)
/* ^^^ I2C config ^^^ */

extern I2C_HandleTypeDef SSH1106_I2C_PORT;

// SSH1106 OLED height in pixels
#ifndef SSH1106_HEIGHT
#define SSH1106_HEIGHT          64
#endif

// SSH1106 width in pixels
#ifndef SSH1106_WIDTH
#define SSH1106_WIDTH           128
#endif

#ifndef SSH1106_BUFFER_SIZE
#define SSH1106_BUFFER_SIZE   SSH1106_WIDTH * SSH1106_HEIGHT / 8
#endif

// Enumeration for screen colors
typedef enum {
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} SSH1106_COLOR;

typedef enum {
    SSH1106_OK = 0x00,
    SSH1106_ERR = 0x01  // Generic error.
} SSH1106_Error_t;

// Struct to store transformations
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
    uint8_t DisplayOn;
} SSH1106_t;
typedef struct {
    uint8_t x;
    uint8_t y;
} SSH1106_VERTEX;

// Procedure definitions
void ssh1106_Init(void);
void ssh1106_Fill(SSH1106_COLOR color);
void ssh1106_UpdateScreen(void);
void ssh1106_DrawPixel(uint8_t x, uint8_t y, SSH1106_COLOR color);
char ssh1106_WriteChar(char ch, FontDef Font, SSH1106_COLOR color);
char ssh1106_WriteString(char* str, FontDef Font, SSH1106_COLOR color);
void ssh1106_SetCursor(uint8_t x, uint8_t y);
void ssh1106_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSH1106_COLOR color);
void ssh1106_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSH1106_COLOR color);
void ssh1106_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSH1106_COLOR color);
void ssh1106_Polyline(const SSH1106_VERTEX *par_vertex, uint16_t par_size, SSH1106_COLOR color);
void ssh1106_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSH1106_COLOR color);
/**
 * @brief Sets the contrast of the display.
 * @param[in] value contrast to set.
 * @note Contrast increases as the value increases.
 * @note RESET = 7Fh.
 */
void ssh1106_SetContrast(const uint8_t value);
/**
 * @brief Set Display ON/OFF.
 * @param[in] on 0 for OFF, any for ON.
 */
void ssh1106_SetDisplayOn(const uint8_t on);
/**
 * @brief Reads DisplayOn state.
 * @return  0: OFF.
 *          1: ON.
 */
uint8_t ssh1106_GetDisplayOn();

// Low-level procedures
void ssh1106_Reset(void);
void ssh1106_WriteCommand(uint8_t byte);
void ssh1106_WriteData(uint8_t* buffer, size_t buff_size);
SSH1106_Error_t ssh1106_FillBuffer(uint8_t* buf, uint32_t len);

_END_STD_C

#endif /* INC_SSH1106_H_ */
