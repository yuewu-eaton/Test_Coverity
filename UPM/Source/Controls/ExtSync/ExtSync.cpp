// ******************************************************************************************************
// *            ExtSync.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: ExtSync.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 10/25/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F28335Port.h"
#include "Eeprom_Map.h"
#include "InverterControl.h"
#include "ParallelCan.h"
#include "Panda28335cfg.h"
#include "ExtSignalPLL.h"
#include <cmath>

// locals
void InitPWMSyncOut( void );
void InitPWMSyncIn( void );

unsigned int     ECapISRCount1 = 0;
unsigned int     ECapDebug1 = 0;
bool PWMSyncReceived = 0;

using namespace std;

extern "C" 
{
    void ECap1_ISR( void );
}    

// ***********************************************************************
// *
// *    FUNCTION: InitHWSync 
// *
// *    DESCRIPTION: Initializes HW ports for external sync signals, configures
// *                 interrupts as required
// *
// *    ARGUMENTS: master true/false
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
//	void InitHWSync( void )
//	{
//	    if ( EEP_IsInternalParallel() )
//	    {
//	        if ( 0 == MyUPMNumber )
//	        {
//	            ConfigureParallelSyncOut( 1 );
//	            InitPWMSyncOut();
//	        }
//	        else
//	        {
//	            ConfigureParallelSyncOut( 0 );
//	            InitPWMSyncIn();
//	        }
//	    }    
//	}

void InitHWSync( void )
{

	static bool bMaster_1 = (~ParallelCan.ParallelStatus.bit.Master);
	if(ParallelCan.ParallelStatus.bit.Master != bMaster_1)
	{
		if(!EnableParalPwmSync)		//no paral sync
		{
		    if ( EEP_IsInternalParallel() )
		    {
		        if ( 0 == MyUPMNumber )
		        {
		            ConfigureParallelSyncOut( 1 );
		            InitPWMSyncOut();
		        }
		        else
		        {
		            ConfigureParallelSyncOut( 0 );
		            InitPWMSyncIn();
		        }
		    } 				        
		}
		else
		{
			if (ParallelCan.ParallelStatus.bit.Master)
	        {
	            ConfigureParallelSyncOut( 1 );
	            InitPWMSyncOut();
	        }
	        else
	        {
	            ConfigureParallelSyncOut( 0 );
	            InitPWMSyncIn();
	        }			
		}
		
	}
	bMaster_1 = ParallelCan.ParallelStatus.bit.Master;

}

// ***********************************************************************
// *
// *    FUNCTION: InitPWMSyncOut 
// *
// *    DESCRIPTION: Initializes GPIO11 as ePWM6B
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InitPWMSyncOut( void )
{
    // Enable ePWM6B as sync out
    // Doesn't call InitEPwm6Gpio() because ePWM6A is already configured as ePWM
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up on GPIO11 (EPWM6B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B
    EDIS;

    // positive edge on timer0
    EPwm6Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm6Regs.AQCTLB.bit.PRD = 0;
    EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;
    EPwm6Regs.AQCTLB.bit.CBU = 0;
    EPwm6Regs.AQCTLB.bit.CAD = 0;
    EPwm6Regs.AQCTLB.bit.CAU = 0;
    EPwm6Regs.CMPB = 150;			//TPR 2884=(150M/26k)/2
}

// ***********************************************************************
// *
// *    FUNCTION: InitPWMSyncIn 
// *
// *    DESCRIPTION: Initializes eCap1 as PWMsync input
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InitPWMSyncIn( void )
{
    // for Denis,20101207
    EALLOW;
//	    GpioCtrlRegs.GPAPUD.bit.GPIO11  = 1;   // Disable pull-up on GPIO11 (EPWM6B)
//	    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;   // Configure GPIO11 as input
//	    GpioCtrlRegs.GPADIR.bit.GPIO11  = 1;   // Input with pull up to set Int_Sync "H"
//	    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;  // Output 0

	GpioCtrlRegs.GPAPUD.bit.GPIO11	= 0;   // enable pull-up on GPIO11 (EPWM6B)
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;   // Configure GPIO11 as io
	GpioCtrlRegs.GPADIR.bit.GPIO11	= 1;   // set Int_Sync as output
	GpioDataRegs.GPASET.bit.GPIO11 = 1;  // Output 0:fail
	DSPOutRegister.GpoA.bit.sync_out_rsd_Low = 1;        
    EDIS;
    // eCAP1 PWMsync in start

    // Configure GPIO
    InitECap1Gpio();

    // 15:14 FREE/SOFT  = 00
    // 13:12 Prescale   = 00
    // 11:9  Prescale   = 000
    // 8     CAPLDEN    = 1    load cap regs
    // 7     CTRRST4    = 0
    // 6     CAP4POL    = 0
    // 5     CTRRST3    = 0
    // 4     CTR3POL    = 0
    // 3     CTRRST2    = 0
    // 2     CTR2POL    = 0
    // 1     CTR1RST    = 0
    // 0     CTR1POL    = 0  rising edge
    ECap1Regs.ECCTL1.all = 0x0100; 

    // 15:12    reserved    = 0000
    // 11       reserved    = 0
    // 10       APWMPOL     = 0
    // 9        CAP/APWM    = 0     capture mode
    // 8        SWSYNC      = 0
    // 7:6      SYNCO_SEL   = 00    pass through
    // 5        SYNCI_EN    = 0     no sync
    // 4        TSCTRSTOP   = 1     free run
    // 3        REARM       = 0
    // 2:1      STOP_WRAP   = 00
    // 0        CONT/ONESHT = 0     
    ECap1Regs.ECCTL2.all = 0x0010;
    
    // clear any spurious interrupts
    ECap1Regs.ECCLR.all = 0xff;

    // Enable capture 1 interrupts
    ECap1Regs.ECEINT.bit.CEVT1 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
    IER |= M_INT4;

    // ecap PWM sync end

    // EXTSYNCI start, this doesn't work but not sure why

//     InitEPwmSyncGpio();
// 
//     EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
}

// ***********************************************************************
// *
// *    FUNCTION: ECap1_ISR 
// *
// *    DESCRIPTION: ECap1 ISR, PWMSync input
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// *    Does not use the dispatcher, no BIOS calls allowed. This ISR also pre-empts
// *    everything except the DMA complete ISR. No additional code is allowed in
// *    this ISR!
// *
// ***********************************************************************
#pragma INTERRUPT( LPI )
#pragma CODE_SECTION( "ramfuncs" )
void ECap1_ISR( void )	//follow 93P
{
	ECapISRCount1++;
	
	// ZhangJun: to show internal sync is ready
	ParallelCan.ParallelStatus.bit.Int_Synced = 1;
	PWMSyncReceived = 1;

   // check that a: not too much time has elapsed since capture event occured and
	// b: the not too big a jump in the ePWM timers, can cause bad gates and HWCL if so
	if((!ParallelCan.ParallelStatus.bit.Master))
	{
		if ( ( ECap1Regs.TSCTR - ECap1Regs.CAP1 ) < 75 )
		{
			if ( EPwm1Regs.TBCTR < 100 )
			{
				ECapDebug1++;
				EPwm1Regs.TBCTL.bit.SWFSYNC = 1;
			}
		}
	}
	ECap1Regs.ECCLR.all = 0xff;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

void FCT_EXT_SYNC_TEST(void)  // Zhangjun : for FCT test
{
    ConfigureParallelSyncOut( 1 );
}

// ******************************************************************************************************
// *            End of PWMSync.cpp
// ******************************************************************************************************
