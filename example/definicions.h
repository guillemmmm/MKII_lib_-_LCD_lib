/*
 * definicions.h
 *
 *  Created on: 1 nov. 2023
 *      Author: guill
 */

#ifndef DEFINICIONS_H_
#define DEFINICIONS_H_

//for turn on UART debigging
//#define UARTdebug

#define longitudLVL 108
#define ampladaLVL 20

// MKII STATE MACHINE

#define IDLE 0x00 //motors parats inici menu (escollir mode+enter)
#define mode2choice1 0x01
#define mode2choice2 0x02
#define mode1 0x03 //we send k params and ideal angle
#define mode2 0x04 //we send DD to apply
#define ready2exit 0x05

//---- end MKII STATTE

struct myADC{
    uint8_t joyX;
    uint8_t joyY;
};

//si 0 255 on 128 es punt mig
#define xLEFTLIM 64
#define xRIGHTLIM 192

//butPressed
#define BUTjoy 0x03
#define BUThigh 0x04
#define BUTlow 0x05

//comunicacions RADIO ------
//modos radio
#define RADIO_modo_normal 0xA3 //modo normal PID on volem controlar angle ideal + param PID
#define RADIO_modo_autoPID 0xE1 //modo on controlem manualment el PID (controlant DD) joystick
#define RADIO_stop 0xF1
#define RADIO_error 0xEE

#define LEN_RADIO_MN 21
#define LEN_RADIO_MA 5

struct RadioCom{
    uint8_t modo; //per indicar mode de funcionament
    uint8_t dades[24]; //24bytes maxim
};

//types of dades to dec data
struct mNTX{        // missatge q envia RADIO -> msp432
    bool changeK; //20 bytes
    float angle;
    float kp;
    float ki;
    float kd;
};
struct mNRX{    // missatge q envia msp432 -> RADIO
    float dutyA;
    float dutyB;
    float error; //error angle desitgat
    uint16_t velA;
    uint16_t velB;
};
struct mATX{    // missatge q envia RADIO -> msp432
    float DD;
};
struct mARX{    // missatge q envia msp432 -> RADIO
    float angle;
    float iangle;
    float dangle;
};


// --- end COM RADIO --------

// -- autoPID alg structs --//
typedef struct {
    float P[3][3];  // Matriz de covarianza
    float theta[3]; // Coeficientes kp,kd,ki
} RLS;


#endif /* DEFINICIONS_H_ */
