//#############################################################################
//
// FILE:   3ChSgenTest.c
//
// TITLE:  Three Phase Power Generator
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
#include "sgen.h"         // Signal Generation Headerfile
#include "driverlib.h"
#include "device.h"
#include <stdio.h>

//
// Three Phase Gen Defines
//
#define REFERENCE_VDAC        0
#define REFERENCE_VREF        1
#define DACA                  1
#define DACB                  2
#define DACC                  3

#define REFERENCE             REFERENCE_VDAC
#define CPUFREQ_MHZ           200

#define LINEAR_INT			  1
#define NORMAL_GEN			  0
#define PHASE_GEN_TYPE	      LINEAR_INT

//
// I2C Defines
//
#define SLAVE_ADDRESS         8
#define BUFFER_SIZE           16    

//
// Operation Codes to Control Work Flow
//
enum opCode
{
    Idle        =   1,
    Start       =   2,
    Stop        =   3,
    PowerDown   =   4
};

//
// Globals
//
enum opCode op_code = Idle; //System starts in Initialize state
volatile struct DAC_REGS* DAC_PTR[4] = {0x0,&DacaRegs,&DacbRegs,&DaccRegs}; //DAC Registers

//Waveform settings
Uint32 samplingFreq_hz  = 200000;
Uint32 outputFreq_hz    = 1000;
Uint32 maxOutputFreq_hz = 5000;
float waveformGain      = 0.8003; // Range 0.0 -> 1.0
float waveformOffset    = 0;      // Range -1.0 -> 1.0

//Define Signal Generator
#if PHASE_GEN_TYPE==LINEAR_INT
SGENTI_3 sgen = SGENTI_3_DEFAULTS;
#elif PHASE_GEN_TYPE==NORMAL_GEN
SGENT_3 sgen = SGENT_3_DEFAULTS;
#endif


//Sgen Channel Value Storage
Uint16 sgen_out1 = 0;
Uint16 sgen_out2 = 0;
Uint16 sgen_out3 = 0;

//I2C Globals
uint16_t rData[BUFFER_SIZE];                 // Send data buffer
//Error Flags
bool RX_ERROR = false;
bool CMD_ERROR = false;


/*//TEMP DATA for master comm testing 
float gain = 0.0;
float offset = 0.0;
uint16_t freq = 1;*/

//Function Prototypes
static inline void setFreq(void);
static inline void setGain(void);
static inline void setOffset(void);
void configureDAC(void);
void configureWaveform(void);
interrupt void cpu_timer0_isr(void);

void initI2CFIFO(void);
inline void decodeMsg(void);   //might be faster to put inline
__interrupt void i2cFIFOISR(void);

//
// Main
//
void main(void)
{
    
//
// Initialize device clock and peripherals
//
    Device_init();

//
// Disable CPU interrupts
//
    DINT;
    
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
    Interrupt_register(INT_TIMER0, &cpu_timer0_isr);
   
    //
    // Set I2C use, initializing it for FIFO mode
    //
    initI2CFIFO();

    //
    // Initialize the data buffers
    //
    uint16_t i;
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        rData[i] = i;   //i for testing. After dev should be 0
    }
//
// Configure DAC
//
    configureDAC();
    
//
// Configure Waveform
//
    configureWaveform();

//
// Initialize Cpu Timers
//
    InitCpuTimers();

//
// Configure Cpu Timer0 to interrupt at specified sampling frequency
//
    ConfigCpuTimer(&CpuTimer0, CPUFREQ_MHZ, 1000000.0/samplingFreq_hz);

//
// Start Cpu Timer0
//
    CpuTimer0Regs.TCR.all = 0x4000;

//
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_I2CA_FIFO);
    //Interrupt_enable(INT_TIMER0);
    
    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    while(1)
    {
        while(op_code == Idle)
        {
            //wait for op_code change from Master
            //
            //delay for 1ms to allow new op_code value to load
            //
            DELAY_US(1000);
        }
        if(op_code == Start){
            Interrupt_enable(INT_TIMER0);
            while(op_code == Start)
            {
                setFreq();   // Set Output Frequency and Max Output Frequency
                setGain();   // Set Magnitude of Waveform
                setOffset(); // Set Offset of Waveform
            }
        }
        if(op_code == Stop){
            //turn off sgen
            //To turn off just set all sgen outputs to 0 and disable timer0 interrupt
            Interrupt_disable(INT_TIMER0);
            //reset DACs
            DAC_PTR[DACA]->DACVALS.all = 0;
	        DAC_PTR[DACB]->DACVALS.all = 0;
	        DAC_PTR[DACC]->DACVALS.all = 0;
            
            //change op_code to idle and wait for further instruction
            op_code = Idle;
        }
        
        if(op_code == PowerDown){
            //turn off sgen
            //To turn off just set all sgen outputs to 0 and disable timer0 interrupt
            Interrupt_disable(INT_TIMER0);
            //reset DACs
            DAC_PTR[DACA]->DACVALS.all = 0;
	        DAC_PTR[DACB]->DACVALS.all = 0;
	        DAC_PTR[DACC]->DACVALS.all = 0;
            //put system in low power mode and wait for wake commnad(Start)
            while(op_code != Start){
                ;
            }
        }
        
	}
}//End Main


//
// setFreq - Set the SINE frequency in SGEN
//
static inline void setFreq(void)
{
	//
    // Range(Q0) = 0x0000 -> 0x7FFF, step_max(Q0) =
    // (Max_Freq_hz*0x10000)/Sampling_Freq_hz
    //
    sgen.step_max = (maxOutputFreq_hz*0x10000)/samplingFreq_hz;

    //
    // Range(Q15) = 0x0000 -> 0x7FFF, freq(Q15) =
    // (Required_Freq_hz/Max_Freq_hz)*0x8000
    //
    sgen.freq = ((float)outputFreq_hz/maxOutputFreq_hz)*0x8000;
}


//
// setGain - Set the gain in SGEN
//
static inline void setGain(void)
{
	sgen.gain = waveformGain * 0x7FFF;   // Range(Q15) = 0x0000 -> 0x7FFF
}


//
// setOffset - Set the offset in SGEN
//
static inline void setOffset(void)
{
	sgen.offset = waveformOffset * 0x7FFF; // Range(Q15) = 0x8000 -> 0x7FFF
}

//
// configureDAC - Enable and configure DAC modules
//
void configureDAC(void)
{
	EALLOW;
	
	//
	//Configure DACA
	//
	DAC_PTR[DACA]->DACCTL.bit.DACREFSEL = REFERENCE;
	DAC_PTR[DACA]->DACOUTEN.bit.DACOUTEN = 1;
	DAC_PTR[DACA]->DACVALS.all = 0;

	//
	//Configure DACB
	//
	DAC_PTR[DACB]->DACCTL.bit.DACREFSEL = REFERENCE;
	DAC_PTR[DACB]->DACOUTEN.bit.DACOUTEN = 1;
	DAC_PTR[DACB]->DACVALS.all = 0;

	//
	//Configure DACC
	//
	DAC_PTR[DACC]->DACCTL.bit.DACREFSEL = REFERENCE;
	DAC_PTR[DACC]->DACOUTEN.bit.DACOUTEN = 1;
	DAC_PTR[DACC]->DACVALS.all = 0;
	
	DELAY_US(10); //Allow for buffered DAC to power up
	
	EDIS;
}

//
// configureWaveform - Configure the SINE waveform
//
void configureWaveform(void)
{
	sgen.alpha = 0; // Range(16) = 0x0000 -> 0xFFFF
    setFreq();
    setGain();
    setOffset();
}

//
// cpu_timer0_isr - Timer ISR that writes the sine value to DAC, log the sine
//                  value, compute the next sine value, and calculate interrupt
//                  duration
//
interrupt void cpu_timer0_isr(void)
{
	//
	// Write current sine value to buffered DACs
	//
	DAC_PTR[DACA]->DACVALS.all = sgen_out1;
	DAC_PTR[DACB]->DACVALS.all = sgen_out2;
	DAC_PTR[DACC]->DACVALS.all = sgen_out3;
	
	//
    // Scale next sine value
    //
    sgen_out1 = (sgen.out1 + 32768) >> 4;
    sgen_out2 = (sgen.out2 + 32768) >> 4;
    sgen_out3 = (sgen.out3 + 32768) >> 4;

	//
    // Compute next sine value
    //
    sgen.calc(&sgen);
	
	//
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
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
    //TX disabled in the hope that everything can be accomplished without
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

       /* //
        // Check received data
        //
        for(i = 0; i < BUFFER_SIZE; i++)
        {
            if(i==0)
            {
                if(rData[i] == 0){
                    //Command to perform handshake
                    op_code = HandShake;
                }
                else if(rData[i] == 7){
                    op_code = Update;
                }
            }
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
        }*/
        
        decodeMsg();
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
    
    outputFreq_hz = ((msbs << 8) | lsbs);
    
    //determine gain
    msbs = rData[6];
    lsbs = rData[7];
    
    temp = ((msbs << 8) | lsbs);
    waveformGain = temp/10.0; //accounts for alterations made during sending from master
    
    //determine offset
    msbs = rData[9];
    lsbs = rData[10];
    temp = ((msbs << 8) | lsbs);
    waveformOffset = temp/10.0;  //accounts for alterations made during sending from master
    
    //determine op_code
    msbs = rData[12];
    lsbs = rData[13];
    temp = ((msbs << 8) | lsbs);
    switch(temp)
    {
        case 1:
            op_code = Idle;
            break;
        case 2:
            op_code = Start;
            break;
        case 3:
            op_code = Stop;
            break;
        case 4:
            op_code = PowerDown;
            break;
        default:
            op_code = Idle;
            break;
    }
}


//
// End of File
//
