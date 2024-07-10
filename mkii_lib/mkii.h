/*
 * mkii.h
 *
 *  Created on: 8 abr. 2024
 *      Author: guill
 */

#ifndef MKII_H_
#define MKII_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "definicions.h"

#include "LCD.h"

#if defined(UARTdebug)

void init_uart(void);
void write(char in[], bool end);
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);

#endif

void init_clock(void);

void init_timers(void);

void init_joystickADC(void);

void init_timer_ADC(bool state);

void joystickISR(void);

void init_buttons(void);

void setButtonInt(bool butJ, bool buttons);

void ButtonISR(void);

//LCD per mes endavant (SPI)
void init_LCD_hw(void);

void init_SPI(void);

void SPI_send(uint8_t count, uint8_t * dataout, uint8_t * datain);

void nRF24ISR(void);

void putData(uint8_t *pointer, uint8_t TxLen); //per carregar dades a nRF24

void delay100us(uint32_t ticks);

void nRF24L01_CE(bool state);

void nRF24L01_CSN(bool state);

void ISRtimerDelay(void);

//state machine

bool message2send(void);

bool waitACK(void);

bool menu2change(void);

uint8_t cNSTATE(uint8_t state);

void updateMenu(uint8_t state);

void RSTmenu2change(void);

void RSTmessage2send(void);

void get_message(uint8_t* payload, uint8_t* LEN, uint8_t state);

void processData(uint8_t* payload);

void updateScreen(uint8_t state, bool full);

void rls_init(RLS *rls);

void rls_update(RLS *rls, float A, float B, float C, float D);

void data2send(uint8_t state); //depenen de l'estat enviem diferents coses

void compute_rls(void);

void rtState(uint8_t state);





#endif /* MKII_H_ */
