// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:19 $
//###########################################################################
//
// FILE:   DSP2833x_EPwm.c
//
// TITLE:  DSP2833x ePWM Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "Constants.h"
#include "Coefficients.h"

// Eaton funcs

#include "InitPWM.h"
void SetupA2DTrigger( volatile struct EPWM_REGS* regs );
void InitEPwmTimer(void);
void InitEPwmChannel( volatile struct EPWM_REGS* regs, uint32_t freq );
void InitAPwm(void);
void InitTZ( void );


//---------------------------------------------------------------------------
// InitEPwm: 
//---------------------------------------------------------------------------
// This function initializes the ePWM(s) to a known state.
//
void InitEPwm(void)
{
   // Initialize ePWM1/2/3/4/5/6

   //tbd...

    // ************* 
    //  Eaton code, not TI
    // *************

    // setting up A2D trigger on ePWM4
    // configure GPIO for eWPM
    InitEPwmGpio();

    SetupA2DTrigger( &EPwm4Regs );              // PWMA: PWM sync out/in, PWMB: A2D mux    
    
    InitEPwmTimer();                            // initialize timer control

    // PWMs active high
    InitEPwmChannel( &EPwm1Regs, PWMFrequency );      // PWMA: Inverter A, PWMB: Rectifier A    
    InitEPwmChannel( &EPwm3Regs, PWMFrequency );      // PWMA: Rectifier B, PWMB: Rectifier C
    InitEPwmChannel( &EPwm5Regs, PWMFrequency );      // PWMA: Inverter B, PWMB: Inverter C
    
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;          // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;        // Clear PWM1A on event A, down count
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;          // Set PWM1B on event B, up count
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;        // Clear PWM1B on event B, down count

    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM2A on event A, up count
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;            // Clear PWM2A on event A, down count
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;              // Set PWM2B on event B, up count
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;            // Clear PWM2B on event B, down count
    
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;              // Set PWM3A on event A, up count
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;            // Clear PWM3A on event A, down count
    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;              // Set PWM3B on event B, up count
    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;            // Clear PWM3B on event B, down count

    // current limits active low
    InitEPwmChannel( &EPwm2Regs, BoostFrequency );  // PWMA/PWMB: Batt
    InitEPwmChannel( &EPwm6Regs, PWMFrequency );  // Programmable current limit- PWMA: Batt, PWMB: spare
    
	//boost pwm5/6 change to duty > 0.5,
    EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;          // set PWM5A on event A, up count
    EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;        // clear PWM5A timer 0
    EPwm5Regs.AQCTLB.bit.CBU = AQ_SET;          // set PWM5B on event B, up down
    EPwm5Regs.AQCTLB.bit.CBD = AQ_CLEAR;        // clear PWM5B on period match
    
    EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;            // Set PWM6A on event A, up count
    EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;          // Clear PWM6A on event A, down count
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear PWM6B on event B, up count
    EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;              // Set PWM6B on event B, down count

    InitAPwm();                                     // interleaved battery legs    
    InitTzGpio();                                   // setup GPIO pins as TZ    
    InitTZ();                                       // configure ePWM TZ    
}

//---------------------------------------------------------------------------
// Example: InitEPwmGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ePWM pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 

void InitEPwmGpio(void)
{
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEPwm3Gpio();
#if DSP28_EPWM4
   InitEPwm4Gpio();       // this one is IO for now anyway
#endif // endif DSP28_EPWM4
#if DSP28_EPWM5    
   InitEPwm5Gpio();
#endif // endif DSP28_EPWM5
#if DSP28_EPWM6
   InitEPwm6Gpio();
#endif // endif DSP28_EPWM6 
}

// HWCL reference PWN, initialize to PWM now
void InitEPwm1Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)   
   
/* Configure ePWM-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;   // Configure GPIO0 as input
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;   // Configure GPIO1 as input
   
    EDIS;
}

// phase B rectiifer/inverter, don't configure as PWM until the legs are turned on
void InitEPwm2Gpio(void)
{
   EALLOW;
    
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM3B)


/* Configure ePWM-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;   // Configure GPIO2 as input
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;   // Configure GPIO3 as input
   
    EDIS;
}

// phase C rectiifer/inverter, don't configure as PWM until the legs are turned on
void InitEPwm3Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
       
/* Configure ePWM-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;   // Configure GPIO4 as input
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;   // Configure GPIO5 as input
    
    EDIS;
}


#if DSP28_EPWM4
// ePWM4B is A/D mux0 line, configure as PWM now
void InitEPwm4Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    //GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWM4A), Puck/20120102, added
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pull-up on GPIO7 (EPWM4B)

/* Configure ePWM-4 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM4 functional pins.
// Comment out other unwanted lines.

    //GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A, Puck/20120102, added
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B
    
    EDIS;
}
#endif // endif DSP28_EPWM4  

// phase A rectiifer/inverter, don't configure as PWM until the legs are turned on
#if DSP28_EPWM5
void InitEPwm5Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pull-up on GPIO9 (EPWM5B)

/* Configure ePWM-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM5 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;   // Configure GPIO9 as EPWM5B
    
    EDIS;
}
#endif // endif DSP28_EPWM5

// ePWM6A is HWCL reference. ePWM6B is PWMSyncOut. Configure ePWM6A now,
// ePWM6B will only be configured as PWM if this is parallel unit and master.
// determined elsewhere
#if DSP28_EPWM6
void InitEPwm6Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;    // Enable pull-up on GPIO10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up on GPIO11 (EPWM6B)

/* Configure ePWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM6 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;    // Configure GPIO10 as input
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;    // Configure GPIO11 as ePWM6B
    
    EDIS;
}
#endif // endif DSP28_EPWM6  

//---------------------------------------------------------------------------
// Example: InitEPwmSyncGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ePWM Synch pins
//

void InitEPwmSyncGpio(void)
{

   EALLOW;

/* Configure EPWMSYNCI  */
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWMSYNCI)
// GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pull-up on GPIO32 (EPWMSYNCI)    

/* Set qualification for selected pins to asynch only */
// This will select synch to SYSCLKOUT for the selected pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;   // Synch to SYSCLKOUT GPIO6 (EPWMSYNCI)
// GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 0;  // Synch to SYSCLKOUT GPIO32 (EPWMSYNCI)    

/* Configure EPwmSync pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPwmSync functional pins.
// Comment out other unwanted lines.   

   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 2;    // Enable pull-up on GPIO6 (EPWMSYNCI)
// GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;   // Enable pull-up on GPIO32 (EPWMSYNCI)    



/* Configure EPWMSYNC0  */

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

// klv: not using EPWMSYNCO, just in

// GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWMSYNC0)
//   GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pull-up on GPIO33 (EPWMSYNC0)    

// GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 3;    // Enable pull-up on GPIO6 (EPWMSYNC0)
//   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2;   // Enable pull-up on GPIO33 (EPWMSYNC0)    
    
    EDIS;
}



//---------------------------------------------------------------------------
// Example: InitTzGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as Trip Zone (TZ) pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 

void InitTzGpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
   GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)
   GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;    // Enable pull-up on GPIO13 (TZ2)
   GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up on GPIO14 (TZ3)
   GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up on GPIO15 (TZ4)

   GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (TZ5)
// GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up on GPIO28 (TZ5)

   GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (TZ6) 
// GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up on GPIO29 (TZ6)  
   
/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

  GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 2;  // Sync, 6 sample qual (TZ1)
  GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 2;  // Sync, 6 sample qual (TZ2)
  GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 2;  // Sync, 6 sample qual (TZ3)
  GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 2;  // Sync, 6 sample qual (TZ4)

  GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 2;  // Sync, 6 sample qual (TZ5)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (TZ5)

  GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 2;  // Sync, 6 sample qual (TZ6) 
// GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;  // Asynch input GPIO29 (TZ6)  

   
/* Configure TZ pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be TZ functional pins.
// Comment out other unwanted lines.   
   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
   GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2
   GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // Configure GPIO14 as TZ3
   GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // Configure GPIO15 as TZ4

   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 3;  // Configure GPIO16 as TZ5
// GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // Configure GPIO28 as TZ5

   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;  // Configure GPIO17 as TZ6               
// GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // Configure GPIO29 as TZ6  

   EDIS;
}

// ***********************************************************************
// *
// *    FUNCTION: SetupA2DTrigger 
// *
// *    DESCRIPTION: Sets up ePWM for ADC trigger, not sure which one
// *                 so passing ePWM struct
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SetupA2DTrigger( volatile struct EPWM_REGS* regs )
{
    // ePWMRegs.TBCTL setup
    // 15:14 emulator mode  = 01        
    // 13 phase dir         = 0
    // 12 clkdiv            = 0         clkdiv = 1
    // 11:10 clkdiv         = 00
    // 9:8 hspclkdiv        = 00        hspclkdiv = 1
    // 7 hspclkdiv          = 0
    // 6 swfsync            = 0
    // 5:4 syncosel         = 11        disabled
    // 3 prdld              = 0         use shadow period register
    // 2 phsen              = 0         no phase register load
    // 1:0 ctrmode          = 00        up count mode
    regs->TBCTL.all = 0x8033;           
    
    // ePWMRegs.CMPCTL setup
    // 15:8: reserved or R/O    = 0000 0000
    // 7 reserved               = 0
    // 6 shdwBmode              = 1     immediate load
    // 5 reserved               = 0
    // 4 shdwAmode              = 0     A shadowed
    // 3:2 loadBmode            = 10    NA for immediate load
    // 1:0 loadAmode            = 10    load on 0 or period match
    regs->CMPCTL.all = 0x004a;

    // ePWMRegs.ETSEL setup cmprb used to trigger adc
    // 15 socBen                = 1     ePWM triggers SOCB
    // 14:12 socBsel            = 001   on timer 0
    // 11 socAen                = 0     no SOCA
    // 10:8 socAsel             = 000   na
    // 7:4 resvd                = 0000
    // 3 inten                  = 0     no ISR
    // 2:0 insel                = 000   NA
    regs->ETSEL.all = 0x9000;

    // ePwmRegs.ETPS setup
    // 15:14 SOCBCNT            = 00    
    // 13:12 SOCBPRD            = 01    generate on 1st event
    // 11:10 SOCACNT            = 00
    // 9:8 SOCAPRD              = 00  
    // 7:4 reserved             = 0000
    // 3:2 INTCNT               = 00
    // 1:0 INTPRD               = 00
    regs->ETPS.all = 0x1000;

    //ePwmRegs.Aqctlb setup
    regs->AQCTLB.bit.CBU = AQ_TOGGLE;
    
    // we add ePwm4A for fan control
    regs->AQCTLA.bit.ZRO = AQ_CLEAR;      // Clear PWM4A on zero, Puck/20120112, added
    regs->AQCTLA.bit.CAU = AQ_SET;        // Set PWM4A on event A, up count, Puck/20120112, added

    // force output low to start
    regs->AQSFRC.bit.RLDCSF = 3;        // immediate load
    regs->AQSFRC.bit.ACTSFB = 1;        // force low
    regs->AQSFRC.bit.OTSFB = 1;         // force a zero

    // clear all event flags
    regs->ETCLR.bit.INT = 1;
    regs->ETCLR.bit.SOCA = 1;
    regs->ETCLR.bit.SOCB = 1;

    // set period
    regs->TBPRD = ( CPUFrequency / ADCFrequency);
//  regs->TBCTR = 0x0000;                           // Clear counter
    regs->TBCTR = 300;                              // Clear counter
        
    // set for 50% duty
    regs->CMPB = ( ( CPUFrequency / ADCFrequency ) / 2 );
}


void InitEPwmTimer(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;

    // Setup Sync
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;      // Sync at counter = 0
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;       // Pass through
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;       // disable EPWMxSYNCO  //??
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;       // Pass through
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;       // Pass through
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;       // Pass through             //??

    // Allow each timer to be sync'ed
    EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;     //not syn, for batt 18k;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;          
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    
    EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;              //count up after sync. signal (EPWM1 Ctr. = 0)
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm4Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm5Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm6Regs.TBCTL.bit.PHSDIR = TB_UP;

    EPwm1Regs.TBPHS.half.TBPHS = 0; 
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm3Regs.TBPHS.half.TBPHS = 0;
    EPwm4Regs.TBPHS.half.TBPHS = 300;
    EPwm5Regs.TBPHS.half.TBPHS = 0;
    EPwm6Regs.TBPHS.half.TBPHS = 14;               //100ns delay for PWMSyncOut
}                                                   


void InitEPwmChannel( volatile struct EPWM_REGS* regs, uint32_t freq  )
{
    // Setup TBCLK
    regs->TBPRD = ( ( CPUFrequency / freq ) / 2 );          
    regs->TBCTR = 0x0000;                           // Clear counter

    // Setup counter mode
    regs->TBCTL.bit.CTRMODE = TB_FREEZE;            // Don't start timer yet
    regs->TBCTL.bit.HSPCLKDIV = TB_DIV1;            // Clock ratio to SYSCLKOUT
    regs->TBCTL.bit.CLKDIV = TB_DIV1;

    // Setup shadowing
    regs->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    regs->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    regs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;   // Load on Zero match
    regs->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

}

void InitAPwm()
{
    // Initialization Time
    //=======================
    // ECAP module 1 config
    ECap4Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    ECap4Regs.ECCTL1.bit.FREE_SOFT = 0;
    ECap5Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    ECap5Regs.ECCTL1.bit.FREE_SOFT = 0;
    ECap6Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    ECap6Regs.ECCTL1.bit.FREE_SOFT = 0;

    ECap4Regs.ECCTL2.bit.CAP_APWM = EC_APWM_MODE;
    ECap4Regs.CAP1 = ( CPUFrequency / PWMFrequency ); // Set period value
    ECap4Regs.CTRPHS = 0; // make eCAP4 reference phase = zero
    ECap4Regs.ECCTL2.bit.APWMPOL = EC_ACTV_HI;
    ECap4Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;         // No sync in for current limits
    ECap4Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCIN; // eCAP4 is Master
    ECap4Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run

    // ECAP module 5 config
    ECap5Regs.ECCTL2.bit.CAP_APWM = EC_APWM_MODE;
    ECap5Regs.CAP1 = ( CPUFrequency / PWMFrequency ); // Set period value
    ECap5Regs.CTRPHS = 0; // Phase offset = 1200-400 = 120 deg
    ECap5Regs.ECCTL2.bit.APWMPOL = EC_ACTV_HI;
    ECap5Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;         // no sync
    ECap5Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCIN; // sync "flow-through"
    ECap5Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run

    // ECAP module 6 config
    ECap6Regs.ECCTL2.bit.CAP_APWM = EC_APWM_MODE;
    ECap6Regs.CAP1 = ( CPUFrequency / PWMFrequency ); // Set period value
    ECap6Regs.CTRPHS = 0; // Phase offset = 1200-400 = 120 deg
    ECap6Regs.ECCTL2.bit.APWMPOL = EC_ACTV_HI;
    ECap6Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;         // no sync
    ECap6Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCIN; // sync "flow-through"
    ECap6Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run
/*
    ECap6Regs.ECCTL2.bit.CAP_APWM = EC_APWM_MODE;
    ECap6Regs.CAP1 = ( CPUFrequency / BoostFrequency ); // Set period value
    ECap6Regs.CTRPHS = 0; // Phase offset = 1200-800 = 240 deg
    ECap6Regs.ECCTL2.bit.APWMPOL = EC_ACTV_LO;
    ECap6Regs.ECCTL2.bit.SYNCI_EN = EC_ENABLE; // slaved off master
    ECap6Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS; // "break the chain"
    ECap6Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; // Allow TSCTR to run
*/     
}

void StartePWMTimers( void )
{
    EALLOW;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;         // Start all the timers synced
    EDIS;
}

void StopePWMTimers( void )
{
    EALLOW;
    
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
    
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;          
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    
    EPwm1Regs.TBCTR = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm3Regs.TBCTR = 0;
    EPwm4Regs.TBCTR = 0;
    EPwm5Regs.TBCTR = 0;
    EPwm6Regs.TBCTR = 0;
    
    EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm4Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm5Regs.TBCTL.bit.PHSDIR = TB_UP;
    EPwm6Regs.TBCTL.bit.PHSDIR = TB_UP;
    
    EPwm1Regs.TBPHS.half.TBPHS = 0; 
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm3Regs.TBPHS.half.TBPHS = 0;
    EPwm4Regs.TBPHS.half.TBPHS = 0;
    EPwm5Regs.TBPHS.half.TBPHS = 0;
    EPwm6Regs.TBPHS.half.TBPHS = 0;
    
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 0;
    SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 0;
    SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 0;
    SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 0;
    SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 0;
    SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 0;
    
    EDIS;
}   
    
// ***********************************************************************
// *
// *    FUNCTION: InitTZ 
// *
// *    DESCRIPTION: Initializes TZ, one-shot no-action taken. Is used
// *                 for indication only, gate is controlled in the PLD
// *                 TZ1 = inverter phA
// *                 TZ2 = inverter phB
// *                 TZ3 = inverter phC
// *                 TZ4 = rectifier phA
// *                 TZ5 = rectifier phB
// *                 TZ6 = rectifier phC
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InitTZ( void )
{
    EALLOW;
    
    // Inverter phA to TZ1, phB to TZ2, phC to TZ3
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;      // one-shot
    //EPwm1Regs.TZSEL.bit.OSHT2 = 1;
    //EPwm1Regs.TZSEL.bit.OSHT3 = 1;
    EPwm1Regs.TZCTL.all = 0x000f;       // no-action for a or b
    EPwm1Regs.TZEINT.all = 0;           // no TZ interrupts
    EPwm1Regs.TZCLR.all = 7;            // clear any spurious flags

    EPwm2Regs.TZSEL.bit.OSHT2 = 1;
    EPwm2Regs.TZCTL.all = 0x000f;       // no-action for a or b
    EPwm2Regs.TZEINT.all = 0;           // no TZ interrupts
    EPwm2Regs.TZCLR.all = 7;            // clear any spurious flags

    EPwm3Regs.TZSEL.bit.OSHT3 = 1;
    EPwm3Regs.TZCTL.all = 0x000f;       // no-action for a or b
    EPwm3Regs.TZEINT.all = 0;           // no TZ interrupts
    EPwm3Regs.TZCLR.all = 7;            // clear any spurious flags

    // Rectifier phA to TZ4, phB to TZ5, phC to TZ6
    EPwm4Regs.TZSEL.bit.OSHT4 = 1;      // one-shot
    //EPwm3Regs.TZSEL.bit.OSHT5 = 1;
    //EPwm3Regs.TZSEL.bit.OSHT6 = 1;
    EPwm4Regs.TZCTL.all = 0x000f;       // no-action for a or b
    EPwm4Regs.TZEINT.all = 0;           // no TZ interrupts
    EPwm4Regs.TZCLR.all = 7;            // clear any spurious flags

    EPwm5Regs.TZSEL.bit.OSHT5 = 1;      // one-shot
    EPwm5Regs.TZCTL.all = 0x000f;       // no-action for a or b
    EPwm5Regs.TZEINT.all = 0;           // no TZ interrupts
    EPwm5Regs.TZCLR.all = 7;            // clear any spurious flags

    EPwm6Regs.TZSEL.bit.OSHT6 = 1;      // one-shot
    EPwm6Regs.TZCTL.all = 0x000f;       // no-action for a or b
    EPwm6Regs.TZEINT.all = 0;           // no TZ interrupts
    EPwm6Regs.TZCLR.all = 7;            // clear any spurious flags

    EDIS;
}


