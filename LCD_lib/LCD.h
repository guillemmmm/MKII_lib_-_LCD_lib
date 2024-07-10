/*
 * LCD.h
 *
 *  Created on: 25 may. 2024
 *      Author: guill
 */

#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

#define ST7735_SLPOUT   (0x11)
#define ST7735_DISPON   (0x29)
#define ST7735_CASET    (0x2A)
#define ST7735_RASET    (0x2B)
#define ST7735_RAMWR    (0x2C)
#define ST7735_RAMRD    (0x2E)
#define ST7735_MADCTL   (0x36)
#define ST7735_IDMOFF   (0x38)
#define ST7735_IDMON    (0x39)
#define ST7735_COLMOD   (0x3A)
#define ST7735_FRMCTR1  (0xB1)
#define ST7735_FRMCTR2  (0xB2)
#define ST7735_FRMCTR3  (0xB3)
#define ST7735_INVCTR   (0xB4)
#define ST7735_PWCTR1   (0xC0)
#define ST7735_PWCTR2   (0xC1)
#define ST7735_PWCTR3   (0xC2)
#define ST7735_PWCTR4   (0xC3)
#define ST7735_PWCTR5   (0xC4)
#define ST7735_VMCTR1   (0xC5)
#define ST7735_GAMCTRP1 (0xE0)
#define ST7735_GAMCTRN1 (0xE1)

#define CLR_RS    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0)
#define SET_RS    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4)
#define CLR_RESET GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0);
#define SET_RESET GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);
#define CLR_CS    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
#define SET_CS    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4)

#define clrLCD Fill_LCD(0x00,0x00,0x00) //ho posem negre

void SPI_sendCommand(uint8_t command);
void SPI_sendData(uint8_t data); //for SPI LCD

void Initialize_LCD(void);

void Set_LCD_for_write_at_X_Y(uint8_t x, uint8_t y);

void Fill_LCD(uint8_t R, uint8_t G, uint8_t B);

void Put_Pixel(uint8_t x, uint8_t y, uint8_t R, uint8_t G, uint8_t B);

void LCD_Circle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t R, uint8_t G, uint8_t B);

uint8_t getFontIndex(char c);

void writeCHAR(uint8_t index, uint8_t sX, uint8_t sY, bool print); //imprimir un char

void writeCHARarray(char cin[], uint8_t x0, uint8_t y0, bool print);

void hsv2rgb(uint16_t h, uint8_t* Re, uint8_t* Gr, uint8_t* Bl);

void printLVL(uint8_t x, uint8_t y, uint8_t lvl); //lvl goes from 0 to 255 (green to red)

void printLVL2(uint8_t x, uint8_t y, uint8_t lvl, uint8_t lvlAnt); //lvl goes from 0 to 255 (green to red)

void floatToString(float number, char *result);

void clearFloatNum(uint8_t x, uint8_t y);

#endif /* LCD_H_ */
