// ********************************************************************************************************
// *            SystemTrap.c
// ********************************************************************************************************
// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON
// *
// ********************************************************************************************************
// *
// *    Copyright (c) 2003...2007 EATON
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: main.c
// *
// *    DESCRIPTION: Contains ISR for system traps
// *
// *    ORIGINATOR: Mike Westerfield
// *
// *    DATE: 7/17/2003
// *
// *    HISTORY: See revision control system's history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"
#include "SystemTrap.h"
#include "I2c_Driver.h"
#include "Alarms.h"
#include "F28335port.h"
#include "Spi_Driver.h"

extern "C"
{
    void InitWatchdog( void );
    void Illegal_isr( void );
    void Watchdog_Overflow_isr( void );
}

// *********************************************************************************************************
// *        Global Variables
// *********************************************************************************************************


//INT16U WatchdogFlags;

// ********************************************************************************************************
// *
// * Function: WatchdogReset(void)
// *
// * Purpose:  Create a watchdog reset
// *
// * Description:  Watchdog reset resets all DSP system registers. 
// *
// ********************************************************************************************************
void WatchdogReset( void )
{
    uint16_t wd_reset_delay = 0;
   
    DINT; // disable interrupts
    
    // WD reset
    EALLOW;
    // Enable watchdog module
    SysCtrlRegs.WDCR = 0x0028;
    // Enable Watchdog reset
    SysCtrlRegs.SCSR = 0x0000;
    // Create WD reset immediately, see jira: GOLDILOCKS-202
    SysCtrlRegs.WDCR = 0x0000; // When bit3-5 is not 101, then it will cause CPU reset

    EDIS;
        
    while (++wd_reset_delay < 1000)
    {
        asm(" nop");
    }
        
    // Should never reach this. Watch dog should kick in.
    // Soft reset
    //Just in case: When soft reset, should clear SP
    asm(" MOV SP,#0");
    asm(" LB #33FFF6h");
}

// ********************************************************************************************************
// *
// * Function: FastShutdown(void);
// *
// * Purpose:  Shutdown everything ASAP
// *
// * Description:
// *
// ********************************************************************************************************
void FastShutdown( void )
{
    //Don't call SetIOGood() function here,because all the interrupts have been disabled.
    //SetIOGood( false );
    GpioDataRegs.GPASET.bit.GPIO24 = 1;
    
    //Don't call FastAuxPowerShutdown() function here, we only need to do some efficient protection.
    //FastAuxPowerShutdown( );
    BoostPWMOff();
    ChargerPWMOff();
    RectifierPWMOff();
    InverterPWMOff();
//    GpioDataRegs.GPCCLEAR.bit.GPIO65 = 0;    //20240910
    
    //Stop all ePWM timers
    StopePWMTimers();
    DMAInitialize();
}

void WriteErrorCodeIntoPld( uint16_t errorcode )
{
    write_PLD_sequence( PLD_ADDR_TRAP, &errorcode);
}

// ********************************************************************************************************
// *
// * Function: InitWatchdog(void);
// *
// * Purpose: Initialize Watchdog
// *
// *
// * Description:  Set Watchdog up to generate interrupt instead of reset.  Counter overflow at approx. 4.4 ms
// *
// ********************************************************************************************************
void InitWatchdog( void )
{
    // Enable watchdog module
    EALLOW;
    SysCtrlRegs.SCSR = 0x0002; //bit 1 == Enable Watchdog interrupt and module
                               //if bit 0 is set watchdog cannot be turned off until hardware reset
    SysCtrlRegs.WDCR = 0x0028; //bit 2-0 : 000 = prescale 1
                               //bit 5-3 : 101 or else immediate reset
                               //bit 6   : watchdog disable
    // Kick Watchdog
    SysCtrlRegs.WDKEY = 0x55;
    SysCtrlRegs.WDKEY = 0xAA;
    EDIS;

}

void DisableWatchdog(void)
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0x55;
    SysCtrlRegs.WDKEY = 0xAA;
    SysCtrlRegs.WDCR  = 0x68;
    EDIS;
}


// ********************************************************************************************************
// *
// * Function: KickWatchdog(void);
// *
// * Purpose: Kick Watchdog
// *
// *
// * Description:  Reset watchdog timer
// *
// ********************************************************************************************************
#pragma CODE_SECTION("ramfuncs");
void KickWatchdog( void )
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0x55;
    SysCtrlRegs.WDKEY = 0xAA;
    EDIS;
}


// ********************************************************************************************************
// *
// * Function: Watchdog_Overflow_isr(void);
// *
// * Purpose: If a watchdog overflow occurs shutdown the rectifier and inverter and reset the dsp.
// *         
// *
// * Description:  A watchdog timeout will occur whenever certain interrupts stop occuring or when the idle
// *               task hasnt happened in 9 ms.
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void Watchdog_Overflow_isr( void )
{
    // step1 : disable interrupt
    DINT;
    // step2 : fast shut down unit
    FastShutdown();
    
    // step3 : write error code to pld
    //Don't re-initialize IIC again, see jira: GOLDILOCKS-202
    WriteErrorCodeIntoPld(WATCHDOG_TRIP);
    //Don't read the error code, see jira: GOLDILOCKS-202
    
    // step4 : trigger watchdog to reset dsp 
    WatchdogReset();
}

// ********************************************************************************************************
// *
// * Function: XNMI_isr(void);
// *
// * Purpose: Shutdown the inverter and recrifier, then reset the dsp if the processor to a known state
// *
// *
// * Description:
// *
// ********************************************************************************************************
/*
interrupt void XNMI_isr(void)
{
    FastShutdown();
    WriteStorageReg(POWER_NMI);
    WatchdogReset();
}
*/

// ********************************************************************************************************
// *
// * Function: Illegal_Address_isr(void);
// *
// * Purpose: Shutdown the inverter and recrifier, then reset the dsp if the processor to a known state
// *          if it has become "lost"
// *
// * Description:
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void Illegal_isr( void )
{
    // step 1 : disable interrupt
    DINT;
    
    // step 2 : fast shut down unit   
    FastShutdown();
    
    // step 3 : write error code to pld
    WriteErrorCodeIntoPld(ILLEGAL_TRAP);
    
    // step 4 : trigger watchdog to reset dsp 
    WatchdogReset();
}

// *********************************************************************************************************
// *        End of File
// *********************************************************************************************************
