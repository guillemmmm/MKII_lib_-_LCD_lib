# nRF24L01
In this repository you'll find the software in C to control the nRF24L01 Radio transmissor. This code is tested and verified in TEXAS INSTRUMENTS DEVICES such as msp432 and TIVA-C.

It is highly recommended to read the nRF24L01 [user guide](https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf) in order to understand how communication works as Nordic has developed its own protocol called **Enhanced ShockBurst**.

If you are not very keen on reading user guides you can go directly to page 47 to understand the different instructions to make the module work.

To operate the module, it can be used as a simple transmitter or receiver (only one of them). However, if a duplex (two-way  data transmission) channel is required, it is recommended to read page 26 to understand the **Enhanced ShockBurst** protocol devoloped by **Nordic**.

In the **lib** folder you'll find the library to work with the nRF24L01 module. And in the **examples** folder a full duplex communication is developed.
