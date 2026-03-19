// TI File $Revision: /main/5 $
// Checkin $Date: October 23, 2007   13:34:09 $
//###########################################################################
//
// FILE:    DSP2833x_Adc.c
//
// TITLE:   DSP2833x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#define ADC_usDELAY  5000ul

// not TI supplied code
void InitProjectAdc( void );

//---------------------------------------------------------------------------
// InitAdc:
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
void InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);


    // *IMPORTANT*
    // The ADC_cal function, which  copies the ADC calibration values from TI reserved
    // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
    // Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
    ADC_cal();
    EDIS;

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
    // after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the DSP2833x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.

    AdcRegs.ADCTRL3.all = 0x00E0;  // Power up bandgap/reference/ADC circuits
    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels

    InitProjectAdc();
}

//---------------------------------------------------------------------------
// InitProjectAdc:
//---------------------------------------------------------------------------
// Set up A/d for Panda project
//
#define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz

void InitProjectAdc( void )
{
    extern void AdcDMAConfigure( void );
    
    // ADCTRL1 setup
    //  AdcRegs.ADCTRL1.bit.bit15 = 0;          // reserved
    //  AdcRegs.ADCTRL1.bit.RESET = 0;          // no reset
    //  AdcRegs.ADCTRL1.bit.SUSMOD = 01;        // stop after current sequence
    // 
    //  AdcRegs.ADCTRL1.bit.ACQ_PS = 0010;      // acquisition size
    // 
    //  AdcRegs.ADCTRL1.bit.CPS = 0;            // pre-scale = 0
    //  AdcRegs.ADCTRL1.bit.CONT_RUN = 0;       // no continuous run
    //  AdcRegs.ADCTRL1.bit.SEQ_OVRD = 0;       // NA for start/stop mode
    //  AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;       // cascaded sequencer mode
    // 
    //  AdcRegs.ADCTRL1.bit.bit0_4 = 0000;      // reserved
    AdcRegs.ADCTRL1.all = 0x1210;

    // ADCTRL2 setup
    //  AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ = 1;      // no SOCB
    //  AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;           // reset to CONV0
    //  AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 0;           // no SOC trigger
    //  AdcRegs.ADCTRL2.bit.rsvd1 = 0;              // reserved bit 12
    // 
    //  AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;       // enable SEQ1 interrupt
    //  AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;       // every SOC event
    //  AdcRegs.ADCTRL2.bit.rsvd2 = 0;              // reserved bit 9
    //  AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0;     // SOC from ePWM
    // 
    //  AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1 = 0;       // no ext trigger
    //  AdcRegs.ADCTRL2.bit.RST_SEQ2 = 0;           // no reset
    //  AdcRegs.ADCTRL2.bit.SOC_SEQ2 = 0;           // no SOC
    //  AdcRegs.ADCTRL2.bit.rsvd3 = 0;              // reserved bit 4
    // 
    //  AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2 = 0;       // no SEQ2 ISR
    //  AdcRegs.ADCTRL2.bit.INT_MOD_SEQ2 = 0;       // na
    //  AdcRegs.ADCTRL2.bit.rsvd4 = 0;              // reserved bit 1
    //  AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2 = 0;     // no SOCB trigger
    AdcRegs.ADCTRL2.all = 0xC800;

    // Need a 25Mhz clock for ADC.  ADC Clock is HSPCLK/2*ADCCLKPS
    AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_MODCLK;

    // conversion registers
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 3;        // IinA/Ibat-   ->A0
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 4;        // IinB/OpaRef  ->A1
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 5;        // IinC/unused  ->A2
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 11;       // IinvA/OutIA  ->A3

    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 12;       // IinvB/OutIB  ->A4
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 13;       // IinvC/OutIC  ->A5
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0;        // VinA/Vbat+   ->B0
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 1;        // VinB/Vbat-   ->B1
    
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 2;        // VinC/Ibat+   ->B2
    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 8;        // VinvA/VbypA  ->B3
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 9;        // VinvB/VbypB  ->B4
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 10;       // VinvC/VbypC  ->B5
    
    AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 6;        // Vrail-/Tinvcap/VGND2/Trectcap ->A6
    AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 14;       // Vrail+/Byp2A/Byp2B/Byp2C ->B6
    AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 15;       // Azero/Trect/Tinv/Tbat/Tsts/P15V/Vbat4/Ref2           ->B7
    AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 7;        // invdcA/invdcB/invdcC/VoutA/VoutB/VoutC/Vbat3/Ref1    ->A7
    
    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 15;       // Set up ADC to perform 1 conversions for every SOC

    // clear SEQ1 and SEQ2
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    AdcRegs.ADCST.bit.INT_SEQ2_CLR = 1;

    // set up DMA
    DMAInitialize();

    // configure adc DMA
    AdcDMAConfigure();
}

//===========================================================================
// End of file.
//===========================================================================
