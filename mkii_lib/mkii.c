/*
 * mkii.c
 *
 *  Created on: 8 abr. 2024
 *      Author: guill
 */

#include "mkii.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include <inc/hw_gpio.h>
#include <inc/hw_types.h>


#include "driverlib/sysctl.h" //libreria del sistema
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "inc/hw_ints.h"
#include "driverlib/uart.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/ssi.h"
#include "inc/hw_ssi.h"
#include "nRF24L01lib.h"

uint8_t payloadRX[32];
uint8_t payloadRXLEN;

uint8_t payloadTX[32];
uint8_t payloadTXLEN;

struct RadioCom myRadioData; //my Com data
struct RadioCom* myRadioDataRxPtr;
struct mNTX     mymNTX; //vars of mySystem
struct mNRX*    mymNRXPtr;
struct mATX     mymATX;
struct mARX*    mymARXPtr;


uint8_t stateSW1[3];

struct myADC myADCdata;

bool changeMenu=false;
bool sendData=false;
bool RADIOint=false;
bool receivedData=false;
uint8_t butPressed=0;

//autoPID alg
RLS rls;
//rls_init(&rls);

#define wl_module_CSN_hi     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
#define wl_module_CSN_lo     GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
#define wl_module_CE_hi      GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
#define wl_module_CE_lo      GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);

#if defined(UARTdebug)
void init_uart(void){
#if defined(init_PORT_E) //si no esta definit
#else
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
#define init_PORT_E
#endif
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
        __nop();
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART7))
    {
        __nop();
    }
    GPIOPinConfigure(GPIO_PE0_U7RX);
    GPIOPinConfigure(GPIO_PE1_U7TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_0);

    UARTClockSourceSet(UART7_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));
}

void write(char in[], bool end)
{
    uint32_t longitud = 0x00;
    while (in[longitud] != 0x00)
    { //per saber longitud caracters
        longitud++;
    }
    UARTSend((uint8_t*) in, longitud);
    if(end){
        UARTSend((uint8_t*) "\n", 1);
    }
}

void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //

    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPut(UART7_BASE , *pui8Buffer++);

    }
}




#endif

void init_clock()
{
    SysCtlClockSet(
//            SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
            SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //50 MHz
    //SysCtlPWMClockSet(SYSCTL_PWMDIV_16); //com volem freq de fins a 160kHz cap a baix, dividim 16MHz/16=1MHz
} //end init_clock

void init_timers(){

    //timer for joystick measurement (TIMER 2)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2))
    {
        __nop();
    }
    TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_PIOSC); //16MHz on 1ms==16000 (16b timer)
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/10);//SysCtlClockGet()/10); //volem 10 mesures/seg
    TimerADCEventSet(TIMER2_BASE, TIMER_ADC_TIMEOUT_A);//que sigui de timeout
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true); //activem timer com a trigger
    //TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT); //en teoria la interrupcio no ha de saltar
    //IntEnable(INT_TIMER2A);
    //----
    //Timer for nRF24lib (period 100us)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1))
    {
        __nop();
    }
    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_PIOSC); //16MHz on 100us==16000000*100*10**-6
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/10000); //100us
    IntRegister(INT_TIMER1A, ISRtimerDelay); //enlacem rutina interrrupcio
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

}

void init_joystickADC(void)
{
#if defined(init_PORT_B) //iniciem portB
#else
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
#define init_PORT_B
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
        __nop();
    }
#endif
#if defined(init_PORT_D) //iniciem portD
#else
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
#define init_PORT_D
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
        __nop();
    }
#endif

    GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_5); //ADC HOR
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3); //ADC VER

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //un ADC per cada port
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
        __nop();
    }

    ADCHardwareOversampleConfigure(ADC0_BASE, 8); //8 mesures de mitjana

    ADCSequenceDisable(ADC0_BASE, 1); //habilitem la sequencia (4 mostres)
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0); //activat per timer 100ms
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0,
                             ADC_CTL_CH11); //interrupt enable, final conversio, canal 8
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1,
                                 ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH4); //interrupt enable, final conversio, canal 8

    ADCSequenceEnable(ADC0_BASE, 1); //habilitem la sequencia

    IntRegister(INT_ADC0SS1, joystickISR); //enllacem rutina interrrupcio
    ADCIntClear(ADC0_BASE, 1);
    ADCIntEnable(ADC0_BASE, 1); //hablitem interrupcions del ADC
    IntEnable(INT_ADC0SS1);
}

void init_timer_ADC(bool state){
    if(state){
        IntEnable(INT_TIMER2A);
        TimerEnable(TIMER2_BASE, TIMER_A);
    } else{
        TimerDisable(TIMER2_BASE, TIMER_A);
        IntDisable(INT_TIMER2A);
    }
}

float parabola(float var , uint8_t mod){
    float res;
    // constant = 0.2/127**2  = 0.0000124
    if(var>128.0f){
        res=(var-128.0f)*(var-128.0f);
    } else{
        res=-(128.0f-var)*(128.0f-var);
    }
    if(mod){
        res*= 0.0000124f; //modo 1 ()
    } else{
        res*= 0.00248f;
    }
    return res;

}


void joystickISR(void)
{
    //mesures adc
    ADCIntClear(ADC0_BASE, 1);
    uint32_t data[4]; //te 4 dades la secuencia
    ADCSequenceDataGet(ADC0_BASE, 1, data); //12 bit ADC (ho passem a 8bit)
    //on data0 es eix 'x' y data y es eix 'y'
    data[0]=data[0]>>4;
    data[1]=data[1]>>4;

    myADCdata.joyX = data[0]; //guardem dades potenciometre
    myADCdata.joyY = data[1];

    sendData=true; //sempre fem sendData

    //tmb aprofitem per saltar flag envio de dades
    uint8_t var = stateSW1[1];
    switch(var){
    case IDLE:{
        if(myADCdata.joyX<xLEFTLIM || myADCdata.joyX>xRIGHTLIM){
            changeMenu=true; //si limit canviem estat
        }
        break;
    }
    case mode2choice1:{
        //
        break;
    }
    case mode2choice2:{
        //
        break;
    }
    case mode1:{ //calculem angle nou a enviar
        float a = (float)data[0]; //on anem de 0.0 a 255.0
        //actualitzem dades struct que enviem
        mymNTX.angle=parabola(a,0);//anem de -40 a 40
        //enviem les k si volem amb change k
        mymNTX.changeK=false;
        break;
    }
    case mode2:{
        float a = (float)data[0]; //on anem de 0.0 a 255.0
        //aqui enviem diferencia de duties DD on pot anar fins a 0.2 maxim (dif de 0.4 de duty)
//        mymATX.DD= -0.2f + a*0.001568627f;
//        mymATX.DD= 0.2f - a*0.001568627f;
        mymATX.DD = parabola(a,1);
        break;
    }
    case ready2exit:{
        break;
    }
    default: break;
    }
}

void init_buttons(void) //but1 =     but2 =
{
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
        __nop();
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    }

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
        __nop();
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    }


    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4); // PE4 es but joystick
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7); // PD6 es but superior

    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY; //unlock PD7
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;

    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7); // PD6 es but superior
                                                                   // PD7 es but inferior
    GPIOIntRegister(GPIO_PORTE_BASE, ButtonISR); //interrupcions de COMPX
    GPIOIntRegister(GPIO_PORTD_BASE, ButtonISR);

    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE); //SWitch de baixada
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7, GPIO_FALLING_EDGE); //SWitch de baixada
//        GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
//    IntEnable(INT_GPIOF);

}

void setButtonInt(bool butJ, bool buttons){
    if(butJ){
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_4);
        IntEnable(INT_GPIOE);
    } else{
        GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_4);
//        IntDisable(INT_GPIOE);
    }
    if(buttons){
        GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7);
        IntEnable(INT_GPIOD);
    } else{
        GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7);
//        IntDisable(INT_GPIOD);
    }
}

void ButtonISR(void)
{

    uint32_t PortE = GPIOIntStatus(GPIO_PORTE_BASE, true);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_4); //PORT F
    uint32_t PortD = GPIOIntStatus(GPIO_PORTD_BASE, true);
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_6|GPIO_INT_PIN_7); //PORT B

    changeMenu=true; //hacremos cambio de menu siempre
    butPressed=0;
    if(PortE==GPIO_INT_PIN_4){
        //joystick button
        butPressed = BUTjoy;
    }
    if((PortD&GPIO_INT_PIN_6)==GPIO_INT_PIN_6){
        //BUT 1
        butPressed = BUThigh;
    }
    if((PortD&GPIO_INT_PIN_7)==GPIO_INT_PIN_7){
        //BUT 2
        butPressed = BUTlow;
    }
}

void init_LCD_hw(void){ //inicialitzacio pins LCD (hardware)
    // cs J2.13 (PA4) , rst J4.17 (PF0) , register select J4.31 (PF4) , backlight J4.39 (PF3)

#if defined(init_PORT_A) //iniciem portE (PE4)
#else
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
#define init_PORT_A
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
        __nop();
    }
#endif
#if defined(init_PORT_F) //iniciem portE (PE4)
#else
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
#define init_PORT_F
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
        __nop();
    }
#endif

    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4); //CS
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0); //rst
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3); //cack light
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4); //RS

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4); //CSn
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_0 | GPIO_PIN_3);// | GPIO_PIN_3);

}

void init_SPI(void){
#if defined(init_PORT_B) //si no esta definit
#else
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //port del SPI PB7 MOSI  PB4 SCLK i MISO PB6
#define init_PORT_B
#endif

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
        __nop();
    }

#if defined(init_PORT_C) //si no esta definit
#else
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //PC6 CE del nRF24 i PC5 nRF24 int
#define init_PORT_C
#endif

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
        __nop();
    }

//#if defined(init_PORT_A) //si no esta definit
//#else
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //PA4 SPI LCD CS i PA3 nRF SPI CS
//#define init_PORT_A
//#endif

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
        __nop();
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    }
#if defined(init_PORT_E) //si no esta definit
#else
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //PA4 SPI LCD CS i PA3 nRF SPI CS
#define init_PORT_E
#endif

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
        __nop();
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2); //posar SPI q es fa servir
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2))
    {
        __nop();
    }

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3); //PA3 SPI_CS
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);  //PC6 nRF24 CE i PC5 int nRF24
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5); //a canviar (no fem servir)

//    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_2); //a canviar (no fem servir)

    //modificacio !!!
    // a la TIVA C del micro del motor el PC5 no va i per tant enllaçem amb PE2

    GPIOIntRegister(GPIO_PORTC_BASE, nRF24ISR);
//    GPIOIntRegister(GPIO_PORTE_BASE, nRF24ISR);

    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE); //SWitch de baixada
//    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE); //SWitch de baixada
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_5); //habilitem ints de nRF24
//    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_2); //habilitem ints de nRF24
    IntEnable(INT_GPIOC);
//    IntEnable(INT_GPIOE);

    wl_module_CE_lo; //posem els pins a estat incial
    wl_module_CSN_hi;


    GPIOPinConfigure(GPIO_PB4_SSI2CLK); //posar els pins q es fan servir
    //    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);

//    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7); //posar pins SPI

    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, //cpol 0 cpha 0
                       SSI_MODE_MASTER, 5000000, 8);  //mode esclau DATA WIDTH 8 a 1MHz (maximum fclk/2) so 5 MHz enough speed
    //mode espia (SLAVE_OD)             //drop to 5MHz due to nRF24

//    IntRegister(INT_SSI2, spiINThandler); //rutina interrupcio SPI (no cal)

//    SSIIntEnable(SSI2_BASE, SSI_TXEOT);

    SSIEnable(SSI2_BASE); //enable SPI module

}

void SPI_send(uint8_t count, uint8_t * dataout, uint8_t * datain){
    bool rec = true;
    if(datain==NULL){
        rec=false;
    }
    uint8_t i,k,j;
    j=0;
    while(1){
        i=0;
        while(SSIDataPutNonBlocking(SSI2_BASE, *(dataout+i+j))) //omplim la TXFIFO
        {
            i++;
            if(((i+j)>=count||(i>=8))){
                break;
            }
        }
        //s'ha omplert TX FIFO
        //esperem a que es buidi
        while((HWREG(SSI2_BASE + SSI_O_SR) & SSI_SR_BSY)); //esperem a que es buidi Tx
//        endSPIt=false;
//        while(!endSPIt);
        uint32_t var;
        uint32_t *RxPointer;
        for (k=0;k<i;k++){
            if(rec){
                RxPointer=(uint32_t*)&(*(datain+k+j));
                SSIDataGetNonBlocking(SSI2_BASE, RxPointer);
            } else{
                SSIDataGetNonBlocking(SSI2_BASE, &var);
            }
            //*(datain+k+j)=dades[i];
        }
        if((j+i)>=count){ //paraules enviades
            break;
        }
        j=j+i; //si reiniciem bucle hem enviat 8 paraules
    }
}


//void spi_transfer_sync (uint8_t * dataout, uint8_t * datain, uint8_t len){
////    uint8_t lenR=len;
//    uint8_t i,k,j;
//    j=0;
//    while(1){
//        i=0;
//        while(SSIDataPutNonBlocking(SSI2_BASE, *(dataout+i))) //omplim la TXFIFO amb -----
//        {
//            i++;
//            if(i>=len){
//                break;
//            }
//        }
//        //s'ha omplert TX FIFO
//        endSPIt=false;
//        while(~endSPIt){ //mentro no estigui buida la txFIFO
//        }
//        for (k=0;k<i;k++){
//            *(datain+k+j)=dades[i];
//        }
//        if((j+i)>=len){ //paraules enviades
//            break;
//        }
//        j=j+8;
//    }
//    /*uint8_t i,j,k,ik;
//    uint8_t lenlst;
//    k=len/8;
//    lenlst=len%8;
//    j=0;
//    for(ik=0;ik<k;ik++){
//        for (i=0;i<8;i++){
//            while(SSIDataPutNonBlocking(SSI2_BASE, *(dataout+i))) //omplim la TXFIFO amb -----
//            {
//            }
//        }
//        endSPIt=false;
//        //ara esperem a que acabi enviament
//        while(~endSPIt){ //mentro no estigui buida la txFIFO
//        }
//        for(i=0;i<8;i++){
//            *(datain+i+8*j)=dades[i];
//        }
//        j++;
//    }
//    //fins 8 paraules
//    for (i=0;i<lenlst;i++){
//        while(SSIDataPutNonBlocking(SSI2_BASE, *(dataout+i))) //omplim la TXFIFO amb -----
//        {
//        }
//    }
//    endSPIt=false;
//    //ara esperem a que acabi enviament
//    while(~endSPIt){ //mentro no estigui buida la txFIFO
//    }
//    for(i=0;i<lenlst;i++){
//        *(datain+i+8*j)=dades[i];
//    }*/
//}
//
//void spi_transmit_sync (uint8_t * dataout, uint8_t len){
//    uint8_t i,j;
//    j=0;
//    while(1){
//        i=0;
//        while(SSIDataPutNonBlocking(SSI2_BASE, *(dataout+i))) //omplim la TXFIFO amb -----
//        {
//            i++;
//            if(i>=len){
//                break;
//            }
//        }
//        //s'ha omplert TX FIFO
//        endSPIt=false;
//        while(~endSPIt){ //mentro no estigui buida la txFIFO
//        }
//        if((j+i)>=len){ //paraules enviades
//            break;
//        }
//        j=j+8;
//    }
//    /*uint8_t i,j,k,ik;
//    uint8_t lenlst;
//    k=len/8;
//    lenlst=len%8;
//    j=0;
//    for(ik=0;ik<k;ik++){
//        for (i=0;i<8;i++){
//            while(SSIDataPutNonBlocking(SSI2_BASE, *(dataout+i))) //omplim la TXFIFO amb -----
//            {
//            }
//        }
//        endSPIt=false;
//        //ara esperem a que acabi enviament
//        while(~endSPIt){ //mentro no estigui buida la txFIFO
//        }
//        j++;
//    }
//    //fins 8 paraules
//    for (i=0;i<lenlst;i++){
//        while(SSIDataPutNonBlocking(SSI2_BASE, *(dataout+i))) //omplim la TXFIFO amb -----
//        {
//        }
//    }
//    endSPIt=false;
//    //ara esperem a que acabi enviament
//    while(~endSPIt){ //mentro no estigui buida la txFIFO
//    }*/
//}
//
//uint8_t spi_fast_shift (uint8_t data){
//    while(SSIDataPutNonBlocking(SSI2_BASE, data)) //omplim la TXFIFO amb -----
//    {
//    }
//    endSPIt=false;
//    //ara esperem a que acabi enviament
//    while(~endSPIt){ //mentro no estigui buida la txFIFO
//    }
//    return dades[0];
//}

void nRF24ISR(void){
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_5);
//    GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_2);

    //netejem CSn per si acas encara ja ho fem en el doci
    nRF24L01_CE(false);

    RADIOint=true; //indiquem que s'ha rebut ACK

}

void nRF24L01_CE(bool state)
{
    if(state){
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
    } else{
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
    }
}

void nRF24L01_CSN(bool state)
{
    if(state){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    } else{
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    }
}

uint32_t TCount;
void delay100us(uint32_t ticks)
{
    TCount=0;
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/10000); //100us
    IntEnable(INT_TIMER1A);
    TimerEnable(TIMER1_BASE, TIMER_A);
    while (TCount < ticks)
    {
    }
    TimerDisable(TIMER1_BASE, TIMER_A);
    IntDisable(INT_TIMER1A);
}

void ISRtimerDelay(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); //netejem flag
    TCount++;
}

bool message2send(void){
    return sendData;
}

bool waitACK(void){
    while(!RADIOint){}
    RADIOint=false;
    uint8_t status;
    status = nRF24L01_nop();  //ho farem mes endavant per no robar recurs SPI

    if((status&nrf24l01_STATUS_TX_DS)==nrf24l01_STATUS_TX_DS){ //si es que s'ha enviat correctament
        //received packet
        return true; //hem rebut dades


        //llegim dada rebuda del ACK
    } else if((status&nrf24l01_STATUS_MAX_RT)==nrf24l01_STATUS_MAX_RT){
        //error de transmissio s'haura de torna a enviar
        return false;
    }
    return false;
}

bool menu2change(void){
    return changeMenu;
}
uint8_t statebefore=0;
uint8_t cNSTATE(uint8_t state){
    uint8_t Nstate=state;

    switch(state){
    case IDLE:{
        if(myADCdata.joyX<xLEFTLIM){
            Nstate=mode2choice2;
#if defined(UARTdebug)
    write("Canviem a estat mode2choice2",true);
#endif
        } else if(myADCdata.joyX>xRIGHTLIM){
            Nstate=mode2choice1;
#if defined(UARTdebug)
    write("Canviem a estat mode2choice1",true);
#endif
        } else{
            Nstate=IDLE;
        }
        break;
    }
    case mode2choice1:{
        if(butPressed==BUThigh){
            butPressed=0;
            Nstate=mode1;
#if defined(UARTdebug)
    write("Canviem a estat mode1",true);
#endif
        } else if(butPressed==BUTlow){
            butPressed=0;
            Nstate=IDLE;
#if defined(UARTdebug)
    write("Tornem a IDLE",true);
#endif
        } else{
            Nstate=mode2choice1;
        }
        break;
    }
    case mode2choice2:{
        if(butPressed==BUThigh){
            butPressed=0;
            Nstate=mode2;
            rls_init(&rls); //iniciem rls
#if defined(UARTdebug)
    write("Canviem a estat mode2",true);
#endif
        } else if(butPressed==BUTlow){
            butPressed=0;
            Nstate=IDLE;
#if defined(UARTdebug)
    write("Tornem al IDLE",true);
#endif
        } else{
            Nstate=mode2choice1;
        }
        break;
    }
    case mode1:{
        if(butPressed==BUTlow){
            butPressed=0;
            Nstate=ready2exit;
            statebefore=mode1;
#if defined(UARTdebug)
    write("Anem a read2exit",true);
#endif
        } else{
            Nstate=mode1;
        }
        break;
    }
    case mode2:{
        if(butPressed==BUTlow){
            butPressed=0;
            Nstate=ready2exit;
            statebefore=mode2;
#if defined(UARTdebug)
    write("Anem a read2exit",true);
#endif
        } else{
            Nstate=mode2;
        }
        break;
    }
    case ready2exit:{
        if(butPressed==BUTjoy){
            Nstate=IDLE;
#if defined(UARTdebug)
    write("Retornem al IDLE",true);
#endif
        } else if(butPressed!=0){
            Nstate=statebefore;
#if defined(UARTdebug)
    write("Retornem a estat anterior",true);
#endif
        } else{
            Nstate=ready2exit;
        }
        break;
    }
    default: break;
    }

    return Nstate;
}

uint8_t stateMenu;

void rtState(uint8_t state){
    stateSW1[1]=state;
}

void updateMenu(uint8_t state){

    stateMenu=state;
    switch(state){
        case IDLE:{
#if defined(UARTdebug)
    write("IDLE state",true);
#endif
            setButtonInt(false, true); //habilitem interrupcions buto y joystick
            init_timer_ADC(true); //prendre mesures i enviar status
            break;
        }
        case mode2choice1:{
#if defined(UARTdebug)
    write("mode2choice1 state",true);
#endif
            //aqui apaguem joy
//            setButtonInt(false, true); //habilitem interrupcions buto y joystick
            init_timer_ADC(false);
            break;
        }
        case mode2choice2:{
#if defined(UARTdebug)
            write("mode2choice2 state",true);
#endif
            //aqui apaguem joy i actualitzem menu
//            setButtonInt(false, true); //habilitem interrupcions buto y joystick
            init_timer_ADC(false);
            break;
        }
        case mode1:{
#if defined(UARTdebug)
    write("mode1 state",true);
#endif
            setButtonInt(false, true); //habilitem interrupcions buto y joystick
            init_timer_ADC(true);
            break;
        }
        case mode2:{
#if defined(UARTdebug)
    write("mode2 state",true);
#endif
            setButtonInt(false, true); //habilitem interrupcions buto y joystick
            init_timer_ADC(true);
            break;
        }
        case ready2exit:{
#if defined(UARTdebug)
    write("ready2exit state",true);
#endif
            setButtonInt(true, true); //habilitem interrupcions buto y joystick
            init_timer_ADC(false);
            break;
        }
        default: break;
        }
}

void RSTmenu2change(void){
    changeMenu=false;
}

void RSTmessage2send(void){
    sendData=false;
}

void get_message(uint8_t* payload, uint8_t* LEN, uint8_t state){
    switch(state){
    case IDLE:{
        //motors parats
        *payload=RADIO_stop;
        *LEN=1;
        break;
    }
    case mode2choice1:{
        //motors parats
        *payload=RADIO_stop;
        *LEN=1;
        break;
    }
    case mode2choice2:{
        //motors parats
        *payload=RADIO_stop;
        *LEN=1;
        break;
    }
    case mode1:{
        *payload=RADIO_modo_normal;
        mymNTX.changeK=false; //no canviem k encara TODO -------------------------------------------------------------------------------------------
        uint8_t i;
        uint8_t *ptr8 = (uint8_t*)&mymNTX;
        for (i=0;i<20;i++){
            *(payload+1+i)=*(ptr8+i);
        }
        *LEN=21;
        break;
    }
    case mode2:{
        *payload=RADIO_modo_autoPID;
        uint8_t i;
        uint8_t *ptr8 = (uint8_t*)&mymATX;
        for (i=0;i<4;i++){
            *(payload+1+i)=*(ptr8+i);
        }
        *LEN=5;
        break;
    }
    case ready2exit:{
        *payload=RADIO_stop;
        *LEN=1;
        break;
    }
    default: break;
    }
}

void processData(uint8_t* payload){
    //aqui hi ha dos opcions o motor parat o depenent del mode rebem dades diferents
    //en teoria modo ha de ser el mateix que li hem enviat
    myRadioDataRxPtr = (struct RadioCom*)payload;
    if(myRadioDataRxPtr->modo==RADIO_stop){
        //no cal fer res
        //imprimirem motor parat com a molt
    } else if(myRadioDataRxPtr->modo==RADIO_modo_normal){
        mymNRXPtr = (struct mNRX*)(myRadioDataRxPtr->dades);
        //canviem imatges si info
    } else if(myRadioDataRxPtr->modo==RADIO_modo_autoPID){ //ojo
        mymARXPtr = (struct mARX*)(myRadioDataRxPtr->dades);
        //canviem imatges si info
    }
}

uint8_t LVL1ant;
uint8_t LVL2ant;

uint8_t Ntimes=0; //1 de cada 10 cops canviem valor (100ms) a (1seg)
void updateScreen(uint8_t state, bool full){ //de moment canviem tota la pantalla per canvi d'estat sino nomes certs param
    switch(state) {
    case IDLE:{
        if(full){
            clrLCD;
            writeCHARarray("IDLE", 40, 100, true);
        } else{
            //actualitzem estat motors (parats en teoria missatge rebut)
            if(myRadioDataRxPtr->modo==RADIO_stop){
                //imprimim motors parats
            } else{
                //imprimim motors encesos
            }
        }
        break;
    }
    case mode2choice1:{
        if(full){
            clrLCD;
            writeCHARarray("CONFIRM", 30, 100, true);
            writeCHARarray("MODE 1", 30, 90, true);
        } else{
            //nthing
        }
        break;
    }
    case mode2choice2:{
        if(full){
            clrLCD;
            writeCHARarray("CONFIRM", 30, 100, true);
            writeCHARarray("MODE 2", 30, 90, true);
        } else{
            //nthing
        }
        break;
    }
    case mode1:{
        if(full){
            clrLCD;
            writeCHARarray("MODE 1", 30, 100, true);
            writeCHARarray("PHI ", 10, 55, true);
            LVL1ant=0;
            LVL2ant=0;

        } else{
            //imprimim duties (min 0.1 maxim 0.4)
            uint8_t var1 = (uint8_t)((255.0)/(0.3)*(mymNRXPtr->dutyA-0.1));
            uint8_t var2 = (uint8_t)((255.0)/(0.3)*(mymNRXPtr->dutyB-0.1));
            printLVL2(1, 10, var1, LVL1ant);
            printLVL2(127-ampladaLVL, 10, var2, LVL2ant);
            LVL1ant=var1;
            LVL2ant=var2;
            //aqui hem de fer update de angle ideal, velocitat A/B , error angle/angle actual ...
            float a0 = mymNRXPtr->error;
            clearFloatNum(50,55);
            char result[10];
            floatToString(a0/100.0, result);
            writeCHARarray(result, 50, 55, true);

        }
        break;
    }
    case mode2:{
        if(full){
            clrLCD;
            writeCHARarray("MODE 2", 30, 100, true);
            writeCHARarray("PHI ", 10, 55, true);
            writeCHARarray("KP ", 15, 40, true);
            writeCHARarray("KD ", 15, 25, true);
            writeCHARarray("KI ", 15, 10, true);
//            LVL1ant=0;
//            LVL2ant=0;
        } else{
//            uint8_t var1 = (uint8_t)((255.0)/(0.3)*(-0.1));
//            uint8_t var2 = (uint8_t)((255.0)/(0.3)*(mymATX.DD-0.1));
//            printLVL2(0, 10, var1, LVL1ant);
//            printLVL2(127-ampladaLVL, 10, var2, LVL2ant);
//            LVL1ant=var1;
//            LVL2ant=var2;

            char result[10];
            //aqui hem de fer update DD actual, error angle/angle actual (rebem), imprimim k's actual fent funcions
            // k's = computePIDparams(DD, error);
            float a0 = mymARXPtr->angle;
            float a1 = mymARXPtr->dangle;
            float a2 = mymARXPtr->iangle;
            rls_update(&rls, mymATX.DD, a0, a1, a2);
            //imprimim a la part inferior les k
            if(Ntimes>3){ //ideally 10
//            if(1)
                clearFloatNum(45,40);
                clearFloatNum(45,25);
                clearFloatNum(45,10);
                clearFloatNum(50,55);
                floatToString(rls.theta[0], result);
                writeCHARarray(result, 45, 40, true);
                floatToString(rls.theta[1], result);
                writeCHARarray(result, 45, 25, true);
                floatToString(rls.theta[2], result);
                writeCHARarray(result, 45, 10, true);
                Ntimes=0;
            }
            else{
                Ntimes++;
            }

            //angle
            clearFloatNum(50,55);
            floatToString(a0/100.0, result);
            writeCHARarray(result, 50, 55, true);

        }
        break;
    }
    case ready2exit:{
        if(full){
            clrLCD;
            writeCHARarray("EXIT", 60, 100, true);
        } else{
            //nthg
        }
        break;
    }
    default: break;
    }
}

void rls_init(RLS *rls) {
    // Inicializem la matriu P com una matriu d identidad gran
    uint8_t i,j;
    float P_init = 1000.0;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            rls->P[i][j] = (i == j) ? P_init : 0.0;
        }
    }

    // theta 0
    for (i = 0; i < 3; ++i) {
        rls->theta[i] = 0.0;
    }
}

// Multiplica una matriz por un vector
void mat_vec_mult(float mat[3][3], float vec[3], float res[3]) {
    uint8_t i,j;
    for (i = 0; i < 3; ++i) {
        res[i] = 0.0f;
        for (j = 0; j < 3; ++j) {
            res[i] += mat[i][j] * vec[j];
        }
    }
}

// Multiplica dos matrices
void mat_mult(float A[3][3], float B[3][3], float res[3][3]) {
    uint8_t i,j,k;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            res[i][j] = 0.0;
            for (k = 0; k < 3; ++k) {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void rls_update(RLS *rls, float A, float B, float C, float D) {
    float phi[3] = {B, C, D};
    float P_phi[3];
//    float phi_T[3];
    float K[3];
    float alpha = 0.0;

    uint8_t i,j;

    // Calcular P * phi
    mat_vec_mult(rls->P, phi, P_phi);

    // Calcular phi^T * P * phi
    for (i = 0; i < 3; ++i) {
        alpha += phi[i] * P_phi[i];
    }

    // Calcular el factor de ganancia K
    for (i = 0; i < 3; ++i) {
        K[i] = P_phi[i] / (1.0 + alpha);
    }

    // Calcular el error de predicción
    float e = A;
    for (i = 0; i < 3; ++i) {
        e -= rls->theta[i] * phi[i];
    }

    // Actualizar los coeficientes theta
    for (i = 0; i < 3; ++i) {
        rls->theta[i] += K[i] * e;
    }

    // Actualizar la matriz P
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            rls->P[i][j] -= K[i] * P_phi[j];
        }
    }
}

void data2send(uint8_t state){

    //esperem a que hi hagin dades a enviar sino sortim
    if(message2send()){

        get_message(payloadTX, &payloadTXLEN, state);

        nRF24L01_flush_tx(); //netejem TX FIFO
        nRF24L01_write_tx_payload_PTX(payloadTX, payloadTXLEN, true); //enviem

        //esperem resposta ACK (int de nRF24) i si true llegim resposta
        if(waitACK()){
            do{
                nRF24L01_payload_width(&payloadRXLEN);
                nRF24L01_read_rx_payload(payloadRX, payloadRXLEN); //i fem processament
            }while(!nRF24L01_FIFO_RX_empty()); //mentres estigui plena (quan buida passa a true)

            //data process
            processData(payloadRX);

            //IMPORTANT, si estem en el modo correcte l'estat ha de coincidir sino no fem res
            if((payloadRX[0]==RADIO_stop)&&(state==IDLE)){
                updateScreen(state, false); //actualitzem pantalla nomes les zones de dades
            } else if((payloadRX[0]==RADIO_modo_normal)&&(state==mode1)){
                updateScreen(state, false); //actualitzem pantalla nomes les zones de dades
            } else if((payloadRX[0]==RADIO_modo_autoPID)&&(state==mode2)){
                updateScreen(state, false); //actualitzem pantalla nomes les zones de dades
            } else{
                //no fem res
            }

        } else{ //MAX_RT no response
            //error goto IDLE
        } //no llegim

        nRF24L01_clear_IRQ(); //netejem interrupts
        RSTmessage2send();

    } else{
        __nop();
    }

    //primer de tot enviem dades en funcio de l'estat
}

void compute_rls(void){
    float a,b,c,d;
    a=0.000784328;
    b=-39.7;
    c=-541.4;
    d=-0.09;
    rls_update(&rls, a,b,c,d);
}

