#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
//#include "driverlib/sysctl.h"


#include "definicions.h"
#include "mkii.h"
#include "nRF24L01lib.h"
#include "LCD.h"

uint8_t payload[32];
uint8_t payloadLEN;

void main(void)
{

    init_clock(); //clock de 16MHz

    init_timers(); //timer mesurajoystick per enviar status (cada 100ms)

    init_joystickADC();

    init_buttons();

#if defined(UARTdebug)
    init_uart();
#endif

//    init_SPI();

    IntMasterEnable(); //habilitem interrupcions

    init_SPI();

//    //fem servir pel triiger el PD1
//    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1); //trigger
//    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7|GPIO_PIN_4); //trigger
//    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
//    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7|GPIO_PIN_4, 0);
//    //configurem MOSI i SCK
//    //Proves gpio posedge
//    while(1){
//
//        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7|GPIO_PIN_4, 0);
//        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
//        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
//        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7|GPIO_PIN_4, GPIO_PIN_7|GPIO_PIN_4);
//        __nop();
//
//    }

    init_LCD_hw();

#if defined(UARTdebug)
    write("Hardware inicialitzat",true);
#endif


    delay100us(1000); //100ms de delay per inicialitzar modul

    nRF24L01_init(); //we use pipe0 communicaiton, default adress
    nRF24L01_powerUP(PTXmode);

    delay100us(20); //1.5 ms d'espera per inicialitzar modul
    nRF24L01_nop();

    Initialize_LCD();

//    writeCHARarray("IDLE", 40, 60, true);

//    Fill_LCD(0xFF,0xFF,0x00); //groc
//    Fill_LCD(0xFF,0x00,0x00); //vermell
//    Fill_LCD(0x00,0x00,0x00); //negre (a 10 kHz tarda uns 45seg (450 000 dades a 128x128 pixels == 27.5 bits/pixel)

//    printLVL(10, 10, 255); //print lvl

    uint8_t R,G,B;
    uint16_t h;
    for (h=0;h<361;h+=60){
        hsv2rgb(h, &R, &G, &B);
        Fill_LCD(R, G, B);
    }

    clrLCD; //pintem de negre

    //Draw a cyan circle
//    LCD_Circle(64, 64, 63,0x00,0xFF,0xFF);
//    LCD_Circle(107, 64, 20,0xFF,0x00,0x00);

    //anem a escriure una lletra 8x8:: ---
    writeCHARarray("IDLE", 40, 100, true); //estat IDLE
    //---//
    ///From here screen changes only done with updateScreen function


    //tenim un timer de 100ms que prepara dades a enviar cap a la msp432

    uint8_t state=IDLE;
#if defined(UARTdebug)
    write("State = IDLE",true);
#endif
    updateScreen(state, true);
    updateMenu(state);

//    setButtonInt(true, true);
//    while(1){
//        get_message(payload, &payloadLEN);
//        //preparem missatge
//        nRF24L01_write_tx_payload_PTX(payload, payloadLEN, true); //enviem
//        __nop();
//        waitACK();
//        nRF24L01_clear_IRQ(); //netejem interrupts
//        __nop();
//    } //prova botons

    //TEST TX MESSAGE
//    while(1){
//        __nop();
//        payload[0]=9;
//        payload[1]=8;
//        payload[2]=7;
//        payload[3]=6;
//        payload[4]=5;
//        payload[5]=4;
//        payloadLEN=6;
//        nRF24L01_write_tx_payload_PTX(payload, payloadLEN, true); //enviem
//
//        if(waitACK()){
//            //                do{
//            nRF24L01_payload_width(&payloadLEN);
//            nRF24L01_read_rx_payload(payload, payloadLEN); //i fem processament
//            nRF24L01_flush_rx();
//            nRF24L01_flush_tx();
//            //                }while(!nRF24L01_FIFO_RX_empty()); //mentres estigui plena (quan buida passa a true)
//        } else{
//        }
//        nRF24L01_clear_IRQ();
//    }

//    char result[32];
//    while(1){
//        writeCHARarray("MODE 2", 30, 100, true);
//        writeCHARarray("KP ", 15, 40, true);
//        writeCHARarray("KD ", 15, 25, true);
//        writeCHARarray("KI ", 15, 10, true);
//
//        clearFloatNum(45,40);
//        clearFloatNum(45,25);
//        clearFloatNum(45,10);
//        floatToString(0.0053, result);
//        writeCHARarray(result, 45, 40, true);
//        floatToString(-1.456, result);
//        writeCHARarray(result, 45, 25, true);
//        floatToString(0.123456, result);
//        writeCHARarray(result, 45, 10, true);
//    }

    //prova rls
//    compute_rls();
//    uint8_t var=0;
//    uint8_t varAnt=var;
//    char result[10];
//    writeCHARarray("p ", ampladaLVL+10, 55, true);
//    floatToString(-1.23, result);
//    writeCHARarray(result, ampladaLVL+10+20, 55, true);
//    while(1){
//        while(var<255){
//            var++;
//            printLVL2(0, 10, var, varAnt);
//            printLVL2(127-ampladaLVL, 10, 255-var, 255-varAnt);
//            delay100us(50); //10ms
//            varAnt=var;
//        }
//        while(var>0){
//            var--;
//            printLVL2(0, 10, var, varAnt);
//            printLVL2(127-ampladaLVL, 10, 255-var, 255-varAnt);
//            delay100us(50); //10ms
//            varAnt=var;
//        }
//    }

    while(1){

        data2send(state); //enviem data si hi ha data per enviar

        if(menu2change()){ //si menu LCD cambia
            state=cNSTATE(state);
            updateMenu(state);
            RSTmenu2change();
            updateScreen(state, true); //actualitzem pantalla
            rtState(state);
        }
        __nop();
    }
} //end main
