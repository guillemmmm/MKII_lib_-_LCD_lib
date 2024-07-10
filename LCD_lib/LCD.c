/*
 * LCD.c
 *
 *  Created on: 25 may. 2024
 *      Author: guill
 */

#include "LCD.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "mkii.h"
//#include "inc/hw_ssi.h"
//#include "driverlib/ssi.h"
//#include "inc/hw_memmap.h"
//#include <inc/hw_types.h>

const uint8_t font8x8_basic[][8] = {
                                      {0x18, 0x24, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42}, // 'A'
                                      {0x7C, 0x42, 0x42, 0x7C, 0x42, 0x42, 0x42, 0x7C}, // 'B'
                                      {0x3C, 0x42, 0x40, 0x40, 0x40, 0x42, 0x42, 0x3C}, // 'C'
                                      {0x7C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x7C}, // 'D'
                                      {0x7E, 0x40, 0x40, 0x7C, 0x40, 0x40, 0x40, 0x7E}, // 'E'
                                      {0x7E, 0x40, 0x40, 0x7C, 0x40, 0x40, 0x40, 0x40}, // 'F'
                                      {0x3C, 0x42, 0x40, 0x40, 0x4E, 0x42, 0x42, 0x3C}, // 'G'
                                      {0x42, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42, 0x42}, // 'H'
                                      {0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C}, // 'I'
                                      {0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x4C, 0x4C, 0x38}, // 'J'
                                      {0x42, 0x44, 0x48, 0x70, 0x48, 0x44, 0x42, 0x42}, // 'K'
                                      {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7E}, // 'L'
                                      {0x42, 0x66, 0x5A, 0x5A, 0x42, 0x42, 0x42, 0x42}, // 'M'
                                      {0x42, 0x62, 0x52, 0x4A, 0x46, 0x42, 0x42, 0x42}, // 'N'
                                      {0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C}, // 'O'
                                      {0x7C, 0x42, 0x42, 0x42, 0x7C, 0x40, 0x40, 0x40}, // 'P'
                                      {0x3C, 0x42, 0x42, 0x42, 0x52, 0x4A, 0x44, 0x3A}, // 'Q'
                                      {0x7C, 0x42, 0x42, 0x42, 0x7C, 0x48, 0x44, 0x42}, // 'R'
                                      {0x3C, 0x42, 0x40, 0x3C, 0x02, 0x42, 0x42, 0x3C}, // 'S'
                                      {0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18}, // 'T'
                                      {0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C}, // 'U'
                                      {0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x24, 0x18}, // 'V'
                                      {0x42, 0x42, 0x42, 0x42, 0x5A, 0x5A, 0x66, 0x42}, // 'W'
                                      {0x42, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x42}, // 'X'
                                      {0x42, 0x42, 0x24, 0x18, 0x18, 0x18, 0x18, 0x18}, // 'Y'
                                      {0x7E, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x7E},  // 'Z'
                                      {0x3C, 0x42, 0x46, 0x4A, 0x52, 0x62, 0x42, 0x3C}, // '0'
                                      {0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C}, // '1'
                                      {0x3C, 0x42, 0x02, 0x1C, 0x20, 0x40, 0x42, 0x7E}, // '2'
                                      {0x3C, 0x42, 0x02, 0x1C, 0x02, 0x02, 0x42, 0x3C}, // '3'
                                      {0x04, 0x0C, 0x14, 0x24, 0x44, 0x7E, 0x04, 0x04}, // '4'
                                      {0x7E, 0x40, 0x40, 0x7C, 0x02, 0x02, 0x42, 0x3C}, // '5'
                                      {0x1C, 0x20, 0x40, 0x7C, 0x42, 0x42, 0x42, 0x3C}, // '6'
                                      {0x7E, 0x02, 0x02, 0x04, 0x08, 0x10, 0x20, 0x20}, // '7'
                                      {0x3C, 0x42, 0x42, 0x3C, 0x42, 0x42, 0x42, 0x3C}, // '8'
                                      {0x3C, 0x42, 0x42, 0x42, 0x3E, 0x02, 0x04, 0x38},  // '9'
                                      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00},    // '.'
                                      {0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00},  // '-'
                                      {0x08, 0x08, 0x3E, 0x6B, 0x49, 0x6B, 0x3E, 0x08}  // Phi num == ','
};

void SPI_sendCommand(uint8_t command){
    CLR_RS;
    CLR_CS;

    SPI_send(1, &command, NULL);

    SET_CS;
}

void SPI_sendData(uint8_t data){
    SET_RS;
    CLR_CS;
    SPI_send(1, &data, NULL);
    SET_CS;
}

void Initialize_LCD(void){
    //Reset the LCD controller
    CLR_RESET;
    delay100us(10);//10µS min
    SET_RESET;
    delay100us(1500);//120mS max

    //SLPOUT (11h): Sleep Out ("Sleep Out"  is chingrish for "wake")
    //The DC/DC converter is enabled, Internal display oscillator
    //is started, and panel scanning is started.
    SPI_sendCommand(ST7735_SLPOUT);
    delay100us(1200);

    //FRMCTR1 (B1h): Frame Rate Control (In normal mode/ Full colors)
    //Set the frame frequency of the full colors normal mode.
    // * Frame rate=fosc/((RTNA + 20) x (LINE + FPA + BPA))
    // * 1 < FPA(front porch) + BPA(back porch) ; Back porch ?0
    //Note: fosc = 333kHz
    SPI_sendCommand(ST7735_FRMCTR1);//In normal mode(Full colors)
    SPI_sendData(0x02);//RTNB: set 1-line period
    SPI_sendData(0x35);//FPB:  front porch
    SPI_sendData(0x36);//BPB:  back porch

    //FRMCTR2 (B2h): Frame Rate Control (In Idle mode/ 8-colors)
    //Set the frame frequency of the Idle mode.
    // * Frame rate=fosc/((RTNB + 20) x (LINE + FPB + BPB))
    // * 1 < FPB(front porch) + BPB(back porch) ; Back porch ?0
    //Note: fosc = 333kHz
    SPI_sendCommand(ST7735_FRMCTR2);//In Idle mode (8-colors)
    SPI_sendData(0x02);//RTNB: set 1-line period
    SPI_sendData(0x35);//FPB:  front porch
    SPI_sendData(0x36);//BPB:  back porch

    //FRMCTR3 (B3h): Frame Rate Control (In Partial mode/ full colors)
    //Set the frame frequency of the Partial mode/ full colors.
    // * 1st parameter to 3rd parameter are used in line inversion mode.
    // * 4th parameter to 6th parameter are used in frame inversion mode.
    // * Frame rate=fosc/((RTNC + 20) x (LINE + FPC + BPC))
    // * 1 < FPC(front porch) + BPC(back porch) ; Back porch ?0
    //Note: fosc = 333kHz
    SPI_sendCommand(ST7735_FRMCTR3);//In partial mode + Full colors
    SPI_sendData(0x02);//RTNC: set 1-line period
    SPI_sendData(0x35);//FPC:  front porch
    SPI_sendData(0x36);//BPC:  back porch
    SPI_sendData(0x02);//RTND: set 1-line period
    SPI_sendData(0x35);//FPD:  front porch
    SPI_sendData(0x36);//BPD:  back porch

    //INVCTR (B4h): Display Inversion Control
    SPI_sendCommand(ST7735_INVCTR);
    SPI_sendData(0x07);
    // 0000 0ABC
    // |||| ||||-- NLC: Inversion setting in full Colors partial mode
    // |||| |||         (0=Line Inversion, 1 = Frame Inversion)
    // |||| |||--- NLB: Inversion setting in idle mode
    // |||| ||          (0=Line Inversion, 1 = Frame Inversion)
    // |||| ||---- NLA: Inversion setting in full Colors normal mode
    // |||| |----- Unused: 0

    //PWCTR1 (C0h): Power Control 1
    SPI_sendCommand(ST7735_PWCTR1);
    SPI_sendData(0x02);// VRH[4:0] (0-31) Sets GVDD
//    SPI_sendData(0x1F);
    // VRH=0x00 => GVDD=5.0v
    // VRH=0x1F => GVDD=3.0v
    // Each tick is a variable step:
    // VRH[4:0] |  VRH | GVDD
    //   00000b | 0x00 | 5.00v
    //   00001b | 0x01 | 4.75v
    //   00010b | 0x02 | 4.70v <<<<<
    //   00011b | 0x03 | 4.65v
    //   00100b | 0x04 | 4.60v
    //   00101b | 0x05 | 4.55v
    //   00110b | 0x06 | 4.50v
    //   00111b | 0x07 | 4.45v
    //   01000b | 0x08 | 4.40v
    //   01001b | 0x09 | 4.35v
    //   01010b | 0x0A | 4.30v
    //   01011b | 0x0B | 4.25v
    //   01100b | 0x0C | 4.20v
    //   01101b | 0x0D | 4.15v
    //   01110b | 0x0E | 4.10v
    //   01111b | 0x0F | 4.05v
    //   10000b | 0x10 | 4.00v
    //   10001b | 0x11 | 3.95v
    //   10010b | 0x12 | 3.90v
    //   10011b | 0x13 | 3.85v
    //   10100b | 0x14 | 3.80v
    //   10101b | 0x15 | 3.75v
    //   10110b | 0x16 | 3.70v
    //   10111b | 0x17 | 3.65v
    //   11000b | 0x18 | 3.60v
    //   11001b | 0x19 | 3.55v
    //   11010b | 0x1A | 3.50v
    //   11011b | 0x1B | 3.45v
    //   11100b | 0x1C | 3.40v
    //   11101b | 0x1D | 3.35v
    //   11110b | 0x1E | 3.25v
    //   11111b | 0x1F | 3.00v
    SPI_sendData(0x02);// 010i i000
    // |||| ||||-- Unused: 0
    // |||| |----- IB_SEL0:
    // ||||------- IB_SEL1:
    // |||-------- Unused: 010
    // IB_SEL[1:0] | IB_SEL | AVDD
    //         00b | 0x00   | 2.5µA   <<<<<
    //         01b | 0x01   | 2.0µA
    //         10b | 0x02   | 1.5µA
    //         11b | 0x03   | 1.0µA

    //PWCTR2 (C1h): Power Control 2
    // * Set the VGH and VGL supply power level
    //Restriction: VGH-VGL <= 32V
    SPI_sendCommand(ST7735_PWCTR2);
    SPI_sendData(0xC5);// BT[2:0] (0-15) Sets GVDD
    // BT[2:0] |    VGH      |     VGL
    //    000b | 4X |  9.80v | -3X |  -7.35v
    //    001b | 4X |  9.80v | -4X |  -9.80v
    //    010b | 5X | 12.25v | -3X |  -7.35v
    //    011b | 5X | 12.25v | -4X |  -9.80v
    //    100b | 5X | 12.25v | -5X | -12.25v
    //    101b | 6X | 14.70v | -3X |  -7.35v   <<<<<
    //    110b | 6X | 14.70v | -4X |  -9.80v
    //    111b | 6X | 14.70v | -5X | -12.25v

    //PWCTR3 (C2h): Power Control 3 (in Normal mode/ Full colors)
    // * Set the amount of current in Operational amplifier in
    //   normal mode/full colors.
    // * Adjust the amount of fixed current from the fixed current
    //   source in the operational amplifier for the source driver.
    // * Set the Booster circuit Step-up cycle in Normal mode/ full
    //   colors.
    SPI_sendCommand(ST7735_PWCTR3);
    SPI_sendData(0x0D);// AP[2:0] Sets Operational Amplifier Bias Current
    // AP[2:0] | Function
    //    000b | Off
    //    001b | Small
    //    010b | Medium Low
    //    011b | Medium
    //    100b | Medium High
    //    101b | Large          <<<<<
    //    110b | reserved
    //    111b | reserved
    SPI_sendData(0x00);// DC[2:0] Booster Frequency
    // DC[2:0] | Circuit 1 | Circuit 2,4
    //    000b | BCLK / 1  | BCLK / 1  <<<<<
    //    001b | BCLK / 1  | BCLK / 2
    //    010b | BCLK / 1  | BCLK / 4
    //    011b | BCLK / 2  | BCLK / 2
    //    100b | BCLK / 2  | BCLK / 4
    //    101b | BCLK / 4  | BCLK / 4
    //    110b | BCLK / 4  | BCLK / 8
    //    111b | BCLK / 4  | BCLK / 16

    //PWCTR4 (C3h): Power Control 4 (in Idle mode/ 8-colors)
    // * Set the amount of current in Operational amplifier in
    //   normal mode/full colors.
    // * Adjust the amount of fixed current from the fixed current
    //   source in the operational amplifier for the source driver.
    // * Set the Booster circuit Step-up cycle in Normal mode/ full
    //   colors.
    SPI_sendCommand(ST7735_PWCTR4);
    SPI_sendData(0x8D);// AP[2:0] Sets Operational Amplifier Bias Current
    // AP[2:0] | Function
    //    000b | Off
    //    001b | Small
    //    010b | Medium Low
    //    011b | Medium
    //    100b | Medium High
    //    101b | Large          <<<<<
    //    110b | reserved
    //    111b | reserved
    SPI_sendData(0x1A);// DC[2:0] Booster Frequency
    // DC[2:0] | Circuit 1 | Circuit 2,4
    //    000b | BCLK / 1  | BCLK / 1
    //    001b | BCLK / 1  | BCLK / 2
    //    010b | BCLK / 1  | BCLK / 4  <<<<<
    //    011b | BCLK / 2  | BCLK / 2
    //    100b | BCLK / 2  | BCLK / 4
    //    101b | BCLK / 4  | BCLK / 4
    //    110b | BCLK / 4  | BCLK / 8
    //    111b | BCLK / 4  | BCLK / 16

    //PPWCTR5 (C4h): Power Control 5 (in Partial mode/ full-colors)
    // * Set the amount of current in Operational amplifier in
    //   normal mode/full colors.
    // * Adjust the amount of fixed current from the fixed current
    //   source in the operational amplifier for the source driver.
    // * Set the Booster circuit Step-up cycle in Normal mode/ full
    //   colors.
    SPI_sendCommand(ST7735_PWCTR5);
    SPI_sendData(0x8D);// AP[2:0] Sets Operational Amplifier Bias Current
    // AP[2:0] | Function
    //    000b | Off
    //    001b | Small
    //    010b | Medium Low
    //    011b | Medium
    //    100b | Medium High
    //    101b | Large          <<<<<
    //    110b | reserved
    //    111b | reserved
    SPI_sendData(0xEE);// DC[2:0] Booster Frequency
    // DC[2:0] | Circuit 1 | Circuit 2,4
    //    000b | BCLK / 1  | BCLK / 1
    //    001b | BCLK / 1  | BCLK / 2
    //    010b | BCLK / 1  | BCLK / 4
    //    011b | BCLK / 2  | BCLK / 2
    //    100b | BCLK / 2  | BCLK / 4
    //    101b | BCLK / 4  | BCLK / 4
    //    110b | BCLK / 4  | BCLK / 8  <<<<<
    //    111b | BCLK / 4  | BCLK / 16

    //VMCTR1 (C5h): VCOM Control 1
    SPI_sendCommand(ST7735_VMCTR1);
    SPI_sendData(0x51);// Default: 0x51 => +4.525
    // VMH[6:0] (0-100) Sets VCOMH
    // VMH=0x00 => VCOMH= +2.5v
    // VMH=0x64 => VCOMH= +5.0v
    SPI_sendData(0x4D);// Default: 0x4D => -0.575
    // VML[6:0] (4-100) Sets VCOML
    // VML=0x04 => VCOML= -2.4v
    // VML=0x64 => VCOML=  0.0v

    //GMCTRP1 (E0h): Gamma ‘+’polarity Correction Characteristics Setting
    SPI_sendCommand(ST7735_GAMCTRP1);
    SPI_sendData(0x0a);
    SPI_sendData(0x1c);
    SPI_sendData(0x0c);
    SPI_sendData(0x14);
    SPI_sendData(0x33);
    SPI_sendData(0x2b);
    SPI_sendData(0x24);
    SPI_sendData(0x28);
    SPI_sendData(0x27);
    SPI_sendData(0x25);
    SPI_sendData(0x2C);
    SPI_sendData(0x39);
    SPI_sendData(0x00);
    SPI_sendData(0x05);
    SPI_sendData(0x03);
    SPI_sendData(0x0d);

    //GMCTRN1 (E1h): Gamma ‘-’polarity Correction Characteristics Setting
    SPI_sendCommand(ST7735_GAMCTRN1);
    SPI_sendData(0x0a);
    SPI_sendData(0x1c);
    SPI_sendData(0x0c);
    SPI_sendData(0x14);
    SPI_sendData(0x33);
    SPI_sendData(0x2b);
    SPI_sendData(0x24);
    SPI_sendData(0x28);
    SPI_sendData(0x27);
    SPI_sendData(0x25);
    SPI_sendData(0x2D);
    SPI_sendData(0x3a);
    SPI_sendData(0x00);
    SPI_sendData(0x05);
    SPI_sendData(0x03);
    SPI_sendData(0x0d);

    //COLMOD (3Ah): Interface Pixel Format
    // * This command is used to define the format of RGB picture
    //   data, which is to be transferred via the MCU interface.
    SPI_sendCommand(ST7735_COLMOD);
    SPI_sendData(0x06);// Default: 0x06 => 18-bit/pixel
    // IFPF[2:0] MCU Interface Color Format
    // IFPF[2:0] | Format
    //      000b | reserved
    //      001b | reserved
    //      010b | reserved
    //      011b | 12-bit/pixel
    //      100b | reserved
    //      101b | 16-bit/pixel
    //      110b | 18-bit/pixel   <<<<<
    //      111b | reserved

    //DISPON (29h): Display On
    // * This command is used to recover from DISPLAY OFF mode. Output
    //   from the Frame Memory is enabled.
    // * This command makes no change of contents of frame memory.
    // * This command does not change any other status.
    // * The delay time between DISPON and DISPOFF needs 120ms at least
    SPI_sendCommand(ST7735_DISPON);//Display On
    delay100us(10);

    //MADCTL (36h): Memory Data Access Control
    SPI_sendCommand(ST7735_MADCTL);
    SPI_sendData(0x40);// YXVL RH-- 0x40
    // |||| ||||-- Unused: 0
    // |||| ||---- MH: Horizontal Refresh Order
    // |||| |        0 = left to right
    // |||| |        1 = right to left
    // |||| |----- RGB: RGB vs BGR Order
    // ||||          0 = RGB color filter panel
    // ||||          1 = BGR color filter panel
    // ||||------- ML: Vertical Refresh Order
    // |||           0 = top to bottom
    // |||           1 = bottom to top
    // |||-------- MV: Row / Column Exchange
    // ||--------- MX: Column Address Order  <<<<<
    // |---------- MY: Row Address Order
}

void Set_LCD_for_write_at_X_Y(uint8_t x, uint8_t y)
{
    //CASET (2Ah): Column Address Set
    // * The value of XS [15:0] and XE [15:0] are referred when RAMWR
    //   command comes.
    // * Each value represents one column line in the Frame Memory.
    // * XS [15:0] always must be equal to or less than XE [15:0]
    SPI_sendCommand(ST7735_CASET); //Column address set
    //Write the parameters for the "column address set" command
    SPI_sendData(0x00);     //Start MSB = XS[15:8]
    SPI_sendData(0x02 + x); //Start LSB = XS[ 7:0]
    SPI_sendData(0x00);     //End MSB   = XE[15:8]
    SPI_sendData(0x81);     //End LSB   = XE[ 7:0]
    //Write the "row address set" command to the LCD
    //RASET (2Bh): Row Address Set
    // * The value of YS [15:0] and YE [15:0] are referred when RAMWR
    //   command comes.
    // * Each value represents one row line in the Frame Memory.
    // * YS [15:0] always must be equal to or less than YE [15:0]
    SPI_sendCommand(ST7735_RASET); //Row address set
    //Write the parameters for the "row address set" command
    SPI_sendData(0x00);     //Start MSB = YS[15:8]
    SPI_sendData(0x01 + y); //Start LSB = YS[ 7:0]
    SPI_sendData(0x00);     //End MSB   = YE[15:8]
    SPI_sendData(0x80);     //End LSB   = YE[ 7:0]
    //Write the "write data" command to the LCD
    //RAMWR (2Ch): Memory Write
    SPI_sendCommand(ST7735_RAMWR); //write data
}

void Fill_LCD(uint8_t R, uint8_t G, uint8_t B)
{
    uint16_t i;
    Set_LCD_for_write_at_X_Y(0, 0);

    //Fill display with a given RGB value
    for (i = 0; i < (128 * 128); i++)
    {
#if defined(UARTdebug)
        if(i%1000==0){
            write("Passem per i=1000",true);
        }
#endif
        SPI_sendData(B); //Blue
        SPI_sendData(G); //Green
        SPI_sendData(R); //Red
    }
}

void Put_Pixel(uint8_t x, uint8_t y, uint8_t R, uint8_t G, uint8_t B)
{
    Set_LCD_for_write_at_X_Y(x, y);
    //Write the single pixel's worth of data
    SPI_sendData(B); //Blue
    SPI_sendData(G); //Green
    SPI_sendData(R); //Red
}

void LCD_Circle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t R, uint8_t G, uint8_t B)
{
    uint8_t x = radius;
    uint8_t y = 0;
    int16_t radiusError = 1 - (int16_t) x;

    while (x >= y)
    {
        //11 O'Clock
        Put_Pixel(x0 - y, y0 + x, R, G, B);
        //1 O'Clock
        Put_Pixel(x0 + y, y0 + x, R, G, B);
        //10 O'Clock
        Put_Pixel(x0 - x, y0 + y, R, G, B);
        //2 O'Clock
        Put_Pixel(x0 + x, y0 + y, R, G, B);
        //8 O'Clock
        Put_Pixel(x0 - x, y0 - y, R, G, B);
        //4 O'Clock
        Put_Pixel(x0 + x, y0 - y, R, G, B);
        //7 O'Clock
        Put_Pixel(x0 - y, y0 - x, R, G, B);
        //5 O'Clock
        Put_Pixel(x0 + y, y0 - x, R, G, B);

        y++;
        if (radiusError < 0)
            radiusError += (int16_t)(2 * y + 1);
        else
        {
            x--;
            radiusError += 2 * (((int16_t) y - (int16_t) x) + 1);
        }
    }
}

uint8_t getFontIndex(char c) {
    if (c >= 0x41 && c <= 0x5A){
        return c-65;
    } else if (c >= '0' && c <= '9'){
        return c - '0' + 26;
    }  //more options modifiable
    else if(c == 0x2E){
        return 36;
    } else if(c == 0x2D){
        return 37;
    } else if(c== 0x70){ // Phi == ','
        return 38;
    }


    else {
        return 0;
    }
}

void writeCHAR(uint8_t index, uint8_t sX, uint8_t sY, bool print){
    sX=sX-4;
    sY=sY-4;

    uint8_t row, col;

    for (row = 0; row < 8; row++) {
        for (col = 0; col < 8; col++) {
//            if (font8x8_basic[index][row] & (1 << (7 - col))) {
            if (font8x8_basic[index][7-row] & (1 << (7 - col))) {
                if(print){
                    Put_Pixel(sX + col, sY + row, 0xFF, 0xFF, 0xFF);
                } else{
                    Put_Pixel(sX + col, sY + row, 0, 0, 0);
                }
//                Put_Pixe{l(sX + col, sY + row, 0xFF, 0xFF, 0xFF);
            }
        }
    }
}


// Función para escribir una cadena (if print true white, if false black)
void writeCHARarray(char cin[], uint8_t x0, uint8_t y0, bool print) {

    uint8_t xi0, yi0,i;
    xi0=x0;
    yi0=y0;
    i=0;
    uint8_t index;

    while(cin[i]!=0x00){
        if(cin[i]==0x20){
        } else{
            index=getFontIndex(cin[i]);
            writeCHAR(index, xi0, yi0, print);
        }
        xi0=xi0+10;
        i++;
    }
}

void hsv2rgb(uint16_t h, uint8_t* Re, uint8_t* Gr, uint8_t* Bl){ //h goes from 0 to 360
    float r, g, b;
    float C = 1.0;  // Dado que S=1 y V=1, entonces C=V*S=1*1=1
    float X = C * (1 - fabs(fmod(h / 60.0, 2) - 1));
    float m = 0.0;  // m=V-C=1-1=0

    if (h < 60) {
        r = C;
        g = X;
        b = 0;
    } else if (60 <= h && h < 120) {
        r = X; g = C; b = 0;
    } else if (120 <= h && h < 180) {
        r = 0; g = C; b = X;
    } else if (180 <= h && h < 240) {
        r = 0; g = X; b = C;
    } else if (240 <= h && h < 300) {
        r = X; g = 0; b = C;
    } else if (300 <= h && h < 360) {
        r = C; g = 0; b = X;
    } else {
        r = 0;
        g = 0;
        b = 0;  // Este caso no debería ocurrir con el valor de H en el rango [0, 360)
    }

    // Convertimos los valores RGB al rango de 0 a 255
    *Re = (uint8_t)((r + m) * 255);
    *Gr = (uint8_t)((g + m) * 255);
    *Bl = (uint8_t)((b + m) * 255);
}

#define lineLEN 25
#define lineW 3
void printLVL(uint8_t x, uint8_t y, uint8_t lvl){
    //primer mirem numero de rayes que volem (10 rayes 100)
    uint8_t yi=y;

    uint8_t rNUM=lvl/10; //paseem de 100->10
    if(rNUM>10) rNUM=10;
    uint8_t i,j,k;
    uint8_t R,G,B;
    B=0;

    float rel=(float)rNUM;
    rel=rel*25.5f;
    G=(uint8_t)rel;

    for (i=0;i<rNUM;i++){
        R=255-G;
        for(k=0;k<lineW;k++){
            Set_LCD_for_write_at_X_Y(x, yi);
            for (j=0;j<lineLEN;j++){
                SPI_sendData(B); //Blue
                SPI_sendData(G); //Green
                SPI_sendData(R); //Red
            }
            yi+=1;
        }
        yi+=4;
        G-=25;
    }
}

void printLVL2(uint8_t x, uint8_t y, uint8_t lvl, uint8_t lvlAnt){
    //define Longitud (Pixels)
    //define Amplada (Pixels)
    uint16_t lvlN = longitudLVL*lvl;
    uint16_t lvlNAnt = longitudLVL*lvlAnt;
    lvlN = lvlN / 256;
    lvlNAnt = lvlNAnt / 256;
    float n;

    uint8_t x0,y0,j,G;

    if(lvlN==lvlNAnt){
        __nop(); //no cal fer res
    } else if(lvlN>lvlNAnt){ //s'ha d'escriure
        x0=x;
        y0=y+lvlNAnt;
        while(lvlNAnt<lvlN){
            Set_LCD_for_write_at_X_Y(x0, y0);
            n = (y0-y)*256;
            n = n/longitudLVL;
            G = (uint8_t)n;
            for (j=0;j<ampladaLVL;j++){
                SPI_sendData(0); //Blue
                SPI_sendData(255-G); //Green
                SPI_sendData(G); //Red
            }
            lvlNAnt++;
            y0++;
        }
//        lvlNAnt--;

    } else{ //s'ha de borrar
        x0=x;
        y0=y+lvlNAnt;
        while(lvlNAnt>lvlN){
            Set_LCD_for_write_at_X_Y(x0, y0);
            for (j=0;j<ampladaLVL;j++){
                SPI_sendData(0); //Blue
                SPI_sendData(0); //Green
                SPI_sendData(0); //Red
            }
            lvlNAnt--;
            y0--;
        }

    }
}

void floatToString(float number, char *result) {
    // Determinar si el número es negativo y tomar el valor absoluto
    int isNegative = number < 0;
    number = fabs(number);

    // Obtener la parte entera
    int integerPart = (int)number;

    // Obtener la parte decimal multiplicando por 100000 y truncando el resultado
    float decimalPart = number - (float)integerPart;
    int decimalInt = (int)(decimalPart * 100000);

    // Ajustar el número para que solo tenga un dígito en la parte entera
    if (integerPart >= 10) {
        integerPart = integerPart % 10;
    }

    // Construir la cadena resultante
    int index = 0;
    result[index++] = '0' + integerPart;

    // Imprimir '-' en vez de '.' si es negativo
    if (isNegative) {
        result[index++] = '-';
    } else {
        result[index++] = '.';
    }

    // Convertir cada uno de los 5 dígitos decimales a caracteres
    int i;
    for (i = 4; i >= 0; i--) {
        result[index + i] = '0' + (decimalInt % 10);
        decimalInt /= 10;
    }

    index += 5;
    result[index] = '\0'; // Terminar la cadena con null
}

void clearFloatNum(uint8_t x, uint8_t y){
    uint8_t i,j;

    for (i=0;i<8;i++){
        Set_LCD_for_write_at_X_Y(x-4, y+i-4);
        for (j=0;j<70;j++){
            SPI_sendData(0); //Blue
            SPI_sendData(0); //Green
            SPI_sendData(0); //Red
        }
    }
}






