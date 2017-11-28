//#############################################################################
//
// FILE:   empty_bitfield_driverlib_main.c
//
// TITLE:  Empty Example
//
// Empty Bit-Field & Driverlib Example
//
// This example is an empty project setup for Bit-Field and Driverlib 
// development.
//
//#############################################################################
// $TI Release: F2837xS Support Library v3.02.00.00 $
// $Release Date: Sat Sep 16 15:30:24 CDT 2017 $
// $Copyright:
// Copyright (C) 2014-2017 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include <stdio.h>

//
// Defines
//
#define SLAVE_ADDRESS   8
#define BUFFER_SIZE 16    

enum opCode
{
    Idle=1,
    HandShake=2,
    Update=3
};

//
// Globals
//
uint16_t handshake[BUFFER_SIZE]={0,0,0,0,0,77,44,25,52,77,53,0,0,0,0,0};
uint16_t sData[BUFFER_SIZE];                 // Send data buffer
uint16_t rData[BUFFER_SIZE];                  // Receive data buffer
uint16_t rDataPoint = 0;            // To keep track of where we are in the
                                    // data stream to check received data
RX_ERROR = false;
CMD_ERROR = false;

//Data from master (watch values)
float gain = 0.0;
float offset = 0.0;
uint16_t freq = 1;

enum opCode op_code = Idle;

//
// Function Prototypes
//
void initI2CFIFO(void);
void decodeMsg(void);
__interrupt void i2cFIFOISR(void);

//
// Main
//
void main(void)
{
    uint16_t i;

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
   
   Interrupt_register(INT_I2CA_FIFO, &i2cFIFOISR);
            
    //
    // Set I2C use, initializing it for FIFO mode
    //
    initI2CFIFO();

    //
    // Initialize the data buffers
    //
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        sData[i] = i;   //i for testing. After dev should be 0
        rData[i] = 0;
    }

    //
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_I2CA_FIFO);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Loop forever. Suspend or place breakpoints to observe the buffers.
    //
    while(1)
    {
        ;
    }
}


//
// Function to configure I2C A in FIFO mode.
//
void initI2CFIFO()
{
    //
    //SDA and SCL pin config
    //
    GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 6);

    //
    // Must put I2C into reset before configuring it
    //
    I2C_disableModule(I2CA_BASE);

    //
    // I2C configuration.
    //
    I2C_setOwnSlaveAddress(I2CA_BASE, SLAVE_ADDRESS);
    I2C_setConfig(I2CA_BASE, I2C_SLAVE_SEND_MODE);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);

    
    //
    // FIFO and interrupt configuration
    //
    I2C_enableFIFO(I2CA_BASE);
    I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF | I2C_INT_TXFF);
    I2C_setFIFOInterruptLevel(I2CA_BASE, I2C_FIFO_TX16, I2C_FIFO_RX16);
    I2C_enableInterrupt(I2CA_BASE, I2C_INT_RXFF);
    //TX disabled until in the hope that everything can be accomplished without
            //I2C_enableInterrupt(I2CA_BASE, I2C_INT_RXFF | I2C_INT_TXFF);

    //
    // Configuration complete. Enable the module.
    //
    I2C_enableModule(I2CA_BASE);
}

//
// I2C A Receive FIFO ISR
//
 __interrupt void i2cFIFOISR(void)
{
    uint16_t i;
    RX_ERROR = false;
    //
    // If receive FIFO interrupt flag is set, read data
    //
    if((I2C_getInterruptStatus(I2CA_BASE) & I2C_INT_RXFF) != 0)
    {
        for(i = 0; i < BUFFER_SIZE; i++)
        {
            rData[i] = I2C_getData(I2CA_BASE);
        }

        //
        // Check received data
        //
        for(i = 0; i < BUFFER_SIZE; i++)
        {
            /*if(i==0)
            {
                if(rData[i] == 0){
                    //Command to perform handshake
                    op_code = HandShake;
                }
                else if(rData[i] == 7){
                    op_code = Update;
                }
            }*/
            if(op_code == HandShake)
            {
                if(rData[i] != handshake[i]){
                    //
                    // Something went wrong. rData doesn't contain expected data.
                    //
                    RX_ERROR = true;
                }
            }
            else if(op_code == Update){
                //Decode rDate buffer and adjust global vars
            }
            else if(op_code == Idle){
                //do nothing with the data
            }
        }
        if (RX_ERROR == false)
        {
            decodeMsg();
        }
        //
        // Clear interrupt flag
        //
        I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF);
    }
   /* //
    // If transmit FIFO interrupt flag is set, put data in the buffer
    //
    else if((I2C_getInterruptStatus(I2CA_BASE) & I2C_INT_TXFF) != 0)
    {
        for(i = 0; i < BUFFER_SIZE; i++)
        {
            I2C_putData(I2CA_BASE, sData[i]);
        }

        //
        // Send the start condition
        //
        I2C_sendStartCondition(I2CA_BASE);

        //
        // Clear interrupt flag
        //
        I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_TXFF);
    }*/

    //
    // Issue ACK
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}

void decodeMsg(void)
{
    //define temp variables 
    uint16_t msbs = 0;
    uint16_t lsbs = 0;
    uint16_t temp = 0;
    
    //determine freq
    msbs = rData[3];
    lsbs = rData[4];
    
    freq = ((msbs << 8) | lsbs);
    
    //determine gain
    msbs = rData[6];
    lsbs = rData[7];
    
    temp = ((msbs << 8) | lsbs);
    gain = temp/10.0; //accounts for alterations made during sending from master
    
    //determine offset
    msbs = rData[9];
    lsbs = rData[10];
    temp = ((msbs << 8) | lsbs);
    offset = temp/10.0;  //accounts for alterations made during sending from master
    
    //determine op_code
    msbs = rData[12];
    lsbs = rData[13];
    temp = ((msbs << 8) | lsbs);
    if(temp==1){
        op_code = Idle;
    }
}

//
// End of File
//

