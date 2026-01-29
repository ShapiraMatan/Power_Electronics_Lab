//#############################################################################
//
// FILE:   adc_ex1_soc_epwm.c
//
// TITLE:  ADC ePWM Triggering
//
// removing my tests 
// testing more of this  
// this is another test from web, testing fetchiung and pulling 
//! \addtogroup bitfield_example_list
//! <h1>ADC ePWM Triggering</h1>
//! this is Anas testing the github repo 
//! This example sets up ePWM1 to periodically trigger a conversion on ADCA.
//!
//! \b External \b Connections \n
//!  - A1 should be connected to a signal to convert
//!
//! \b Watch \b Variables \n
//! - \b adcAResults - A sequence of analog-to-digital conversion samples from
//!   pin A1. The time between samples is determined based on the period
//!   of the ePWM timer.
//!
//
//#############################################################################
//
//
// 
// C2000Ware v6.00.01.00
//
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
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
#include "f28x_project.h"

//
// Defines
//
#define RESULTS_BUFFER_SIZE     256

//
// Globals
//
uint16_t adcAResults[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t index;                              // Index into result buffer
volatile uint16_t bufferFull;                // Flag to indicate buffer is full


// Define the wave parameter 
// 100MHz / (2 * 100kHz) = 500
#define PWM_PRD_VAL       500  
#define WAVEFORM_TYPE     2     // 2 = Up-Down Count mode

// Global variable for Duty Cycle (0.0 to 1.0)
volatile float dutyCycle = 0.5f;


//
// Function Prototypes
//
void initADC(void);
void initEPWM(void);
void UpdateDuty(void); 
void initADCSOC(void);
__interrupt void adcA1ISR(void);

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    InitSysCtrl();

    //
    // Initialize GPIO
    //
    InitGpio();

        // 2. Configure GPIO Muxing for ePWM7
    EALLOW;
    // Set Pin 12 to EPWM7A (GMUX = 0, MUX = 1 for F28P55x)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;
    
    // Set Pin 13 to EPWM7B (GMUX = 0, MUX = 1 for F28P55x)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;
    EDIS;

    //
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    InitPieVectTable();

    //
    // Map ISR functions
    //
    EALLOW;
    PieVectTable.ADCA1_INT = &adcA1ISR;     // Function for ADCA interrupt 1
    EDIS;

    //
    // Configure the ADC and power it up
    //
    initADC();

    //
    // Configure the ePWM
    //
        // 4. Configure ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all PWM clocks for sync
    EDIS;

    initEPWM();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all PWM clocks
    EDIS;

    // Setup the ADC for ePWM triggered conversions on channel 1
    //
    initADCSOC();

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1;  // Enable group 1 interrupts

    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM

    //
    // Initialize results buffer
    //
    for(index = 0; index < RESULTS_BUFFER_SIZE; index++)
    {
        adcAResults[index] = 0;
    }

    index = 0;
    bufferFull = 0;

    //
    // Enable PIE interrupt
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    //
    // Sync ePWM
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    //
    // Take conversions indefinitely in loop
    //
    while(1)
    {
        
        // Commented as per lab instruction 1/23/2026
        // //
        // // Start ePWM
        // //
        // EPwm1Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOCA
        // EPwm1Regs.TBCTL.bit.CTRMODE = 0;   // Unfreeze, and enter up count mode

        UpdateDuty(); // added this function 
        DELAY_US(1000); // adding delay 

        // //
        // // Wait while ePWM causes ADC conversions, which then cause interrupts,
        // // which fill the results buffer, eventually setting the bufferFull
        // // flag
        // //
        while(!bufferFull)
        {
        }
        bufferFull = 0; //clear the buffer full flag

        //
        // Stop ePWM
        //
        EPwm1Regs.ETSEL.bit.SOCAEN = 0;    // Disable SOCA
        EPwm1Regs.TBCTL.bit.CTRMODE = 3;   // Freeze counter

        //
        // Software breakpoint. At this point, conversion results are stored in
        // adcAResults.
        //
        // Hit run again to get updated conversions.
        //
        ESTOP0;
    }
}

//
// initADC - Function to configure and power up ADCA.
//
void initADC(void)
{
    //
    // Setup VREF as internal
    //
    SetVREF(ADC_ADCA, ADC_INTERNAL, ADC_VREF3P3);

    EALLOW;

    //
    // Set ADCCLK divider to /4
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;

    //
    // Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Power up the ADC and then delay for 1 ms
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;

    DELAY_US(1000);
}

//
// initEPWM - Function to configure ePWM1 to generate the SOC.
//
// float dutyCycle = 0.5f; // the duty cycle 
// #define WAVEFORM_TYPE TB_COUNT_UP;  // waveform type 
/*
Defined (PWM_Carrier_Waveform) waveforms types:  
TB_COUNT_UP (0x0)
TB_COUNT_DOWN (0x1)
TB_COUNT_UPDOWN (0x2)
TB_FREEZE (0x3)
*/

// 100MHz / (2 * 100kHz) = 500
// #define PWM_PRD_VAL 500  
// float PWM_frequency = 100e3; // 1/Ts = 1/(NrTclk) = switching frequency 
// float SysClockFreq = 1.5e6;


void initEPWM(void)
{
    //uint32_t period;
    EALLOW;
    // // Start all PWM clocks
    // 0 (Stop) / 1 (Start)
    //CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Do we really need this? 



    // Setting up the carrier wave parameters 
    // --- Time Base (TB) Subsystem ---
    EPwm7Regs.TBPRD = PWM_PRD_VAL;           // Set period
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;      // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;                // Clear counter
    EPwm7Regs.TBCTL.bit.CTRMODE = WAVEFORM_TYPE; // Up-Down Count
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = 0;       // TBCLK = SYSCLKOUT / 1
    EPwm7Regs.TBCTL.bit.CLKDIV = 0;

    // --- Counter Compare (CC) Subsystem ---
    EPwm7Regs.CMPA.bit.CMPA = PWM_PRD_VAL / 2; // Default 50% duty

    // --- Action Qualifier (AQ) Subsystem ---
    // Actions for EPWM7A
    EPwm7Regs.AQCTLA.bit.CAU = 2; // Set High on CMPA Up-count
    EPwm7Regs.AQCTLA.bit.CAD = 1; // Set Low on CMPA Down-count

    // --- Dead-Band (DB) Subsystem ---
    // Configure for Active High Complementary (AHC)
    EPwm7Regs.DBCTL.bit.OUT_MODE = 3;  // Fully enabled RED and FED
    EPwm7Regs.DBCTL.bit.POLSEL = 2;    // Active High Complementary (B is inverted A)
    EPwm7Regs.DBCTL.bit.IN_MODE = 0;   // EPWM7A is the source for both delays
    
    EPwm7Regs.DBRED.bit.DBRED = 20;    // 200ns delay at 100MHz
    EPwm7Regs.DBFED.bit.DBFED = 20;    // 200ns delay at 100MHz






/* Commented for now (01/29/2026)

    EPwm1Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event
    EPwm1Regs.CMPA.bit.CMPA = 0x0800;   // Set compare A value to 2048 counts
    EPwm1Regs.TBPRD = 0x1000;           // Set period to 4096 counts
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;    // Freeze counter

    EPwm1Regs.TBCTL.bit.CLKDIV = 000;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 000;
    EPwm1Regs.TBCTL.bit.CTRMODE = 00;
    EPwm7Regs.TBPRD = (uint16_t)period;
    EPwm1Regs.CMPA.bit.CMPA = (uint16_t)(period * 1.0f - PWM_Duty_Cycle);
    EPwm1Regs.(AQCTLA, AQCTLB) .bit.(CAD, CAU, ZRO, PRD)

    // AQCTL: Action Qualifier (Complementary Logic)
    if (carrierType == CC_COUNT_UPDOWN) {
        // PWMA Logic
        EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;   // Set on increment match
        EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear on decrement match
        
        // PWMB Logic (Inverse of A)
        EPwm7Regs.AQCTLB.bit.CAU = AQ_CLEAR; 
        EPwm7Regs.AQCTLB.bit.CAD = AQ_SET;   
    } 
    else if (carrierType == CC_COUNT_UP) {
        EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set at zero
        EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear at match
        
        EPwm7Regs.AQCTLB.bit.ZRO = AQ_CLEAR; // Clear at zero
        EPwm7Regs.AQCTLB.bit.CAU = AQ_SET;   // Set at match
    }
    else { // CC_COUNT_DOWN
        EPwm7Regs.AQCTLA.bit.PRD = AQ_SET;   // Set at period
        EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear at match
        
        EPwm7Regs.AQCTLB.bit.PRD = AQ_CLEAR; 
        EPwm7Regs.AQCTLB.bit.CAD = AQ_SET;   
    }

    // Enable TBCLK sync
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
*/
    EDIS;
}

// adding this function to control duty cycle 
void UpdateDuty(void)
{
    // Clamp duty cycle for safety
    if(dutyCycle > 0.95f) dutyCycle = 0.95f; 
    if(dutyCycle < 0.05f) dutyCycle = 0.05f;

    // In Up-Down mode, duty cycle is inverse to CMPA value
    // High CMPA = Small Duty cycle; Low CMPA = Large Duty cycle
    uint16_t newCmpA = (uint16_t)((1.0f - dutyCycle) * (float)PWM_PRD_VAL);
    
    EPwm7Regs.CMPA.bit.CMPA = newCmpA;
}
//
// initADCSOC - Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{
    //
    // Select the channels to convert and the end of conversion flag
    //
    EALLOW;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;     // SOC0 will convert pin A1
                                           // 0:A0  1:A1  2:A2  3:A3
                                           // 4:A4   5:A5   6:A6   7:A7
                                           // 8:A8   9:A9   A:A10  B:A11
                                           // C:A12  D:A13  E:A14  F:A15
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9;     // Sample window is 10 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // Trigger on ePWM1 SOCA

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    EDIS;
}

//
// adcA1ISR - ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{
    //
    // Add the latest result to the buffer
    // ADCRESULT0 is the result register of SOC0
    adcAResults[index++] = AdcaResultRegs.ADCRESULT0;

    //
    // Set the bufferFull flag if the buffer is full
    //
    if(RESULTS_BUFFER_SIZE <= index)
    {
        index = 0;
        bufferFull = 1;
    }

    //
    // Clear the interrupt flag
    //
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    //
    // Check if overflow has occurred
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }

    //
    // Acknowledge the interrupt
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of File
//
