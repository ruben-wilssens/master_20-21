/*
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 * https://github.com/afiskon/stm32-ssh1106
 * Adapted by Victor Van der Elst & Ruben Wilssens in 2021
 *
 */

#include <math.h>
#include <ssh1106.h>
#include <stdlib.h>
#include <string.h>  // For memcpy

void ssh1106_Reset(void) {
    /* for I2C - do nothing */
}

// Send a byte to the command register
void ssh1106_WriteCommand(uint8_t byte) {
    HAL_I2C_Mem_Write(&SSH1106_I2C_PORT, SSH1106_I2C_ADDR, 0x00, 1, &byte, 1, HAL_MAX_DELAY);
}

// Send data
void ssh1106_WriteData(uint8_t* buffer, size_t buff_size) {
    HAL_I2C_Mem_Write(&SSH1106_I2C_PORT, SSH1106_I2C_ADDR, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}

// Screenbuffer
static uint8_t SSH1106_Buffer[SSH1106_BUFFER_SIZE];

// Screen object
static SSH1106_t SSH1106;

// Fills the Screenbuffer with values from a given buffer of a fixed length
SSH1106_Error_t ssh1106_FillBuffer(uint8_t* buf, uint32_t len) {
    SSH1106_Error_t ret = SSH1106_ERR;
    if (len <= SSH1106_BUFFER_SIZE) {
        memcpy(SSH1106_Buffer,buf,len);
        ret = SSH1106_OK;
    }
    return ret;
}

// Initialize the oled screen
void ssh1106_Init(void) {
    // Reset OLED
    ssh1106_Reset();

    // Wait for the screen to boot
    HAL_Delay(100);

    // Init OLED
    ssh1106_SetDisplayOn(0); //display off

    ssh1106_WriteCommand(0x20); //Set Memory Addressing Mode
    ssh1106_WriteCommand(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                                // 10b,Page Addressing Mode (RESET); 11b,Invalid

    ssh1106_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

#ifdef SSH1106_MIRROR_VERT
    ssh1106_WriteCommand(0xC0); // Mirror vertically
#else
    ssh1106_WriteCommand(0xC8); //Set COM Output Scan Direction
#endif

    ssh1106_WriteCommand(0x00); //---set low column address
    ssh1106_WriteCommand(0x10); //---set high column address

    ssh1106_WriteCommand(0x40); //--set start line address - CHECK

    ssh1106_SetContrast(0xFF);

#ifdef SSH1106_MIRROR_HORIZ
    ssh1106_WriteCommand(0xA0); // Mirror horizontally
#else
    ssh1106_WriteCommand(0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef SSH1106_INVERSE_COLOR
    ssh1106_WriteCommand(0xA7); //--set inverse color
#else
    ssh1106_WriteCommand(0xA6); //--set normal color
#endif

// Set multiplex ratio.
#if (SSH1106_HEIGHT == 128)
    // Found in the Luma Python lib for SH1106.
    ssh1106_WriteCommand(0xFF);
#else
    ssh1106_WriteCommand(0xA8); //--set multiplex ratio(1 to 64) - CHECK
#endif

#if (SSH1106_HEIGHT == 32)
    ssh1106_WriteCommand(0x1F); //
#elif (SSH1106_HEIGHT == 64)
    ssh1106_WriteCommand(0x3F); //
#elif (SSH1106_HEIGHT == 128)
    ssh1106_WriteCommand(0x3F); // Seems to work for 128px high displays too.
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    ssh1106_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    ssh1106_WriteCommand(0xD3); //-set display offset - CHECK
    ssh1106_WriteCommand(0x00); //-not offset

    ssh1106_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
    ssh1106_WriteCommand(0xF0); //--set divide ratio

    ssh1106_WriteCommand(0xD9); //--set pre-charge period
    ssh1106_WriteCommand(0x22); //

    ssh1106_WriteCommand(0xDA); //--set com pins hardware configuration - CHECK
#if (SSH1106_HEIGHT == 32)
    ssh1106_WriteCommand(0x02);
#elif (SSH1106_HEIGHT == 64)
    ssh1106_WriteCommand(0x12);
#elif (SSH1106_HEIGHT == 128)
    ssh1106_WriteCommand(0x12);
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    ssh1106_WriteCommand(0xDB); //--set vcomh
    ssh1106_WriteCommand(0x20); //0x20,0.77xVcc

    ssh1106_WriteCommand(0x8D); //--set DC-DC enable
    ssh1106_WriteCommand(0x14); //
    ssh1106_SetDisplayOn(1); //--turn on SSH1106 panel

    // Clear screen
    ssh1106_Fill(Black);

    // Flush buffer to screen
    ssh1106_UpdateScreen();

    // Set default values for screen object
    SSH1106.CurrentX = 0;
    SSH1106.CurrentY = 0;

    SSH1106.Initialized = 1;
}

// Fill the whole screen with the given color
void ssh1106_Fill(SSH1106_COLOR color) {
    /* Set memory */
    uint32_t i;

    for(i = 0; i < sizeof(SSH1106_Buffer); i++) {
        SSH1106_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

// Write the screenbuffer with changed to the screen
void ssh1106_UpdateScreen(void) {
    // Write data to each page of RAM. Number of pages
    // depends on the screen height:
    //
    //  * 32px   ==  4 pages
    //  * 64px   ==  8 pages
    //  * 128px  ==  16 pages
    for(uint8_t i = 0; i < SSH1106_HEIGHT/8; i++) {
        ssh1106_WriteCommand(0xB0 + i); // Set the current RAM page address.
        ssh1106_WriteCommand(0x00);
        ssh1106_WriteCommand(0x10);
        ssh1106_WriteData(&SSH1106_Buffer[SSH1106_WIDTH*i],SSH1106_WIDTH);
    }
}

//    Draw one pixel in the screenbuffer
//    X => X Coordinate
//    Y => Y Coordinate
//    color => Pixel color
void ssh1106_DrawPixel(uint8_t x, uint8_t y, SSH1106_COLOR color) {
    if(x >= SSH1106_WIDTH || y >= SSH1106_HEIGHT) {
        // Don't write outside the buffer
        return;
    }

    // Check if pixel should be inverted
    if(SSH1106.Inverted) {
        color = (SSH1106_COLOR)!color;
    }

    // Draw in the right color
    if(color == White) {
        SSH1106_Buffer[x + (y / 8) * SSH1106_WIDTH] |= 1 << (y % 8);
    } else {
        SSH1106_Buffer[x + (y / 8) * SSH1106_WIDTH] &= ~(1 << (y % 8));
    }
}

// Draw 1 char to the screen buffer
// ch       => char om weg te schrijven
// Font     => Font waarmee we gaan schrijven
// color    => Black or White
char ssh1106_WriteChar(char ch, FontDef Font, SSH1106_COLOR color) {
    uint32_t i, b, j;

    // Check if character is valid
    if (ch < 32 || ch > 126)
        return 0;

    // Check remaining space on current line
    if (SSH1106_WIDTH < (SSH1106.CurrentX + Font.FontWidth) ||
        SSH1106_HEIGHT < (SSH1106.CurrentY + Font.FontHeight)){
        // Not enough space on current line
        return 0;
    }

    // Use the font to write
    for(i = 0; i < Font.FontHeight; i++) {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for(j = 0; j < Font.FontWidth; j++) {
            if((b << j) & 0x8000)  {
                ssh1106_DrawPixel(SSH1106.CurrentX + j, (SSH1106.CurrentY + i), (SSH1106_COLOR) color);
            }else{
                ssh1106_DrawPixel(SSH1106.CurrentX + j, (SSH1106.CurrentY + i), (SSH1106_COLOR)!color);
            }
        }
    }

    // The current space is now taken
    SSH1106.CurrentX += Font.FontWidth;

    // Return written char for validation
    return ch;
}

// Write full string to screenbuffer
char ssh1106_WriteString(char* str, FontDef Font, SSH1106_COLOR color) {
    // Write until null-byte
    while (*str) {
        if (ssh1106_WriteChar(*str, Font, color) != *str) {
            // Char could not be written
            return *str;
        }

        // Next char
        str++;
    }

    // Everything ok
    return *str;
}

// Position the cursor
void ssh1106_SetCursor(uint8_t x, uint8_t y) {
    SSH1106.CurrentX = x;
    SSH1106.CurrentY = y;
}

// Draw line by Bresenhem's algorithm
void ssh1106_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSH1106_COLOR color) {
  int32_t deltaX = abs(x2 - x1);
  int32_t deltaY = abs(y2 - y1);
  int32_t signX = ((x1 < x2) ? 1 : -1);
  int32_t signY = ((y1 < y2) ? 1 : -1);
  int32_t error = deltaX - deltaY;
  int32_t error2;

  ssh1106_DrawPixel(x2, y2, color);
  while((x1 != x2) || (y1 != y2)){
    ssh1106_DrawPixel(x1, y1, color);
    error2 = error * 2;
    if(error2 > -deltaY){
      error -= deltaY;
      x1 += signX;
    }else{
    /*nothing to do*/
    }

    if(error2 < deltaX){
      error += deltaX;
      y1 += signY;
    }else{
    /*nothing to do*/
    }
  }
  return;
}
//Draw polyline
void ssh1106_Polyline(const SSH1106_VERTEX *par_vertex, uint16_t par_size, SSH1106_COLOR color) {
  uint16_t i;
  if(par_vertex != 0){
    for(i = 1; i < par_size; i++){
      ssh1106_Line(par_vertex[i - 1].x, par_vertex[i - 1].y, par_vertex[i].x, par_vertex[i].y, color);
    }
  }else{
    /*nothing to do*/
  }
  return;
}
/*Convert Degrees to Radians*/
static float ssh1106_DegToRad(float par_deg) {
    return par_deg * 3.14 / 180.0;
}

/*Normalize degree to [0;360]*/
static uint16_t ssh1106_NormalizeTo0_360(uint16_t par_deg) {
  uint16_t loc_angle;
  if(par_deg <= 360){
    loc_angle = par_deg;
  }else{
    loc_angle = par_deg % 360;
    loc_angle = ((par_deg != 0)?par_deg:360);
  }
  return loc_angle;
}

/* DrawArc. Draw angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle in degree
 * sweep in degree
 */
void ssh1106_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSH1106_COLOR color) {
    #define CIRCLE_APPROXIMATION_SEGMENTS 36
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1,xp2;
    uint8_t yp1,yp2;
    uint32_t count = 0;
    uint32_t loc_sweep = 0;
    float rad;

    loc_sweep = ssh1106_NormalizeTo0_360(sweep);

    count = (ssh1106_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree = loc_sweep / (float)approx_segments;
    while(count < approx_segments){
        rad = ssh1106_DegToRad(count*approx_degree);
        xp1 = x + (int8_t)(sin(rad)*radius);
        yp1 = y + (int8_t)(cos(rad)*radius);
        count++;
        if(count != approx_segments){
            rad = ssh1106_DegToRad(count*approx_degree);
        }else{
            rad = ssh1106_DegToRad(loc_sweep);
        }
        xp2 = x + (int8_t)(sin(rad)*radius);
        yp2 = y + (int8_t)(cos(rad)*radius);
        ssh1106_Line(xp1,yp1,xp2,yp2,color);
    }

    return;
}
//Draw circle by Bresenhem's algorithm
void ssh1106_DrawCircle(uint8_t par_x,uint8_t par_y,uint8_t par_r,SSH1106_COLOR par_color) {
  int32_t x = -par_r;
  int32_t y = 0;
  int32_t err = 2 - 2 * par_r;
  int32_t e2;

  if (par_x >= SSH1106_WIDTH || par_y >= SSH1106_HEIGHT) {
    return;
  }

    do {
      ssh1106_DrawPixel(par_x - x, par_y + y, par_color);
      ssh1106_DrawPixel(par_x + x, par_y + y, par_color);
      ssh1106_DrawPixel(par_x + x, par_y - y, par_color);
      ssh1106_DrawPixel(par_x - x, par_y - y, par_color);
        e2 = err;
        if (e2 <= y) {
            y++;
            err = err + (y * 2 + 1);
            if(-x == y && e2 <= x) {
              e2 = 0;
            }
            else{
              /*nothing to do*/
            }
        }else{
          /*nothing to do*/
        }
        if(e2 > x){
          x++;
          err = err + (x * 2 + 1);
        }else{
          /*nothing to do*/
        }
    } while(x <= 0);

    return;
}

//Draw rectangle
void ssh1106_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSH1106_COLOR color) {
  ssh1106_Line(x1,y1,x2,y1,color);
  ssh1106_Line(x2,y1,x2,y2,color);
  ssh1106_Line(x2,y2,x1,y2,color);
  ssh1106_Line(x1,y2,x1,y1,color);

  return;
}

void ssh1106_SetContrast(const uint8_t value) {
    const uint8_t kSetContrastControlRegister = 0x81;
    ssh1106_WriteCommand(kSetContrastControlRegister);
    ssh1106_WriteCommand(value);
}

void ssh1106_SetDisplayOn(const uint8_t on) {
    uint8_t value;
    if (on){
        value = 0xAF;   // Display on
        SSH1106.DisplayOn = 1;
    }else{
        value = 0xAE;   // Display off
        SSH1106.DisplayOn = 0;
    }
    ssh1106_WriteCommand(value);
}

uint8_t ssh1106_GetDisplayOn() {
    return SSH1106.DisplayOn;
}
